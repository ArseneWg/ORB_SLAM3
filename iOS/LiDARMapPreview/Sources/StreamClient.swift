import ARKit
import CoreImage
import Foundation
import Network
import UIKit

final class StreamClient: @unchecked Sendable {
    struct Endpoint: Equatable {
        var host: String
        var port: UInt16
    }

    private let stateLock = NSLock()
    private let sendQueue = DispatchQueue(label: "LiDARMapPreview.StreamClient")
    private let ciContext = CIContext()

    private var endpoint = Endpoint(host: "", port: 9000)
    private var connection: NWConnection?
    private var isReady = false
    private var isStreaming = false
    private var frameIndex: UInt64 = 0
    private var lastSentTimestamp: TimeInterval = 0
    private let targetFPS: Double = 10.0

    var onStatusChanged: ((String) -> Void)?
    var onFrameCountChanged: ((UInt64) -> Void)?

    func start(host: String, port: UInt16) {
        let trimmedHost = host.trimmingCharacters(in: .whitespacesAndNewlines)
        guard !trimmedHost.isEmpty else {
            publishStatus("enter Mac IP")
            return
        }

        let newEndpoint = Endpoint(host: trimmedHost, port: port)
        var shouldReconnect = false

        stateLock.lock()
        if endpoint != newEndpoint || connection == nil {
            endpoint = newEndpoint
            shouldReconnect = true
        }
        isStreaming = true
        stateLock.unlock()

        if shouldReconnect {
            reconnect()
        } else {
            publishStatus("streaming to \(trimmedHost):\(port)")
        }
    }

    func stop() {
        stateLock.lock()
        isStreaming = false
        isReady = false
        let currentConnection = connection
        connection = nil
        stateLock.unlock()

        currentConnection?.cancel()
        publishStatus("stopped")
    }

    func process(frame: ARFrame) {
        let snapshot: (Endpoint, UInt64)?

        stateLock.lock()
        let active = isStreaming && isReady && connection != nil
        let minInterval = 1.0 / targetFPS
        let enoughTimeElapsed = frame.timestamp - lastSentTimestamp >= minInterval

        if active && enoughTimeElapsed {
            lastSentTimestamp = frame.timestamp
            frameIndex += 1
            snapshot = (endpoint, frameIndex)
        } else {
            snapshot = nil
        }
        stateLock.unlock()

        guard let (currentEndpoint, currentFrameIndex) = snapshot else {
            return
        }

        guard let payload = makePayload(frame: frame, frameIndex: currentFrameIndex) else {
            return
        }

        sendQueue.async { [weak self] in
            guard let self else { return }

            self.stateLock.lock()
            let currentConnection = self.connection
            self.stateLock.unlock()

            currentConnection?.send(content: payload, completion: .contentProcessed { error in
                if let error {
                    self.publishStatus("send failed: \(error.localizedDescription)")
                    self.reconnect()
                    return
                }

                DispatchQueue.main.async {
                    self.onFrameCountChanged?(currentFrameIndex)
                }
                self.publishStatus("streaming to \(currentEndpoint.host):\(currentEndpoint.port)")
            })
        }
    }
}

private extension StreamClient {
    struct PacketHeader: Codable {
        let version: Int
        let frameIndex: UInt64
        let timestamp: Double
        let displayOrientation: String
        let rgbWidth: Int
        let rgbHeight: Int
        let depthWidth: Int
        let depthHeight: Int
        let fx: Float
        let fy: Float
        let cx: Float
        let cy: Float
        let pose: [Float]
        let rgbSize: Int
        let depthSize: Int
    }

    func reconnect() {
        stateLock.lock()
        let currentEndpoint = endpoint
        let shouldStream = isStreaming

        connection?.cancel()
        connection = nil
        isReady = false
        stateLock.unlock()

        guard shouldStream else {
            return
        }

        guard let nwPort = NWEndpoint.Port(rawValue: currentEndpoint.port) else {
            publishStatus("invalid port")
            return
        }

        publishStatus("connecting to \(currentEndpoint.host):\(currentEndpoint.port)")

        let connection = NWConnection(
            host: NWEndpoint.Host(currentEndpoint.host),
            port: nwPort,
            using: .tcp
        )

        connection.stateUpdateHandler = { [weak self] state in
            guard let self else { return }

            switch state {
            case .ready:
                self.stateLock.lock()
                self.isReady = true
                self.stateLock.unlock()
                self.publishStatus("connected to \(currentEndpoint.host):\(currentEndpoint.port)")

            case .waiting(let error):
                self.stateLock.lock()
                self.isReady = false
                self.stateLock.unlock()
                self.publishStatus("waiting: \(error.localizedDescription)")

            case .failed(let error):
                self.stateLock.lock()
                self.isReady = false
                self.connection = nil
                self.stateLock.unlock()
                self.publishStatus("failed: \(error.localizedDescription)")

            case .cancelled:
                self.stateLock.lock()
                self.isReady = false
                self.connection = nil
                self.stateLock.unlock()

            case .setup, .preparing:
                self.publishStatus("connecting to \(currentEndpoint.host):\(currentEndpoint.port)")

            @unknown default:
                self.publishStatus("unknown connection state")
            }
        }

        stateLock.lock()
        self.connection = connection
        stateLock.unlock()

        connection.start(queue: sendQueue)
    }

    func publishStatus(_ status: String) {
        DispatchQueue.main.async { [weak self] in
            self?.onStatusChanged?(status)
        }
    }

    func makePayload(frame: ARFrame, frameIndex: UInt64) -> Data? {
        guard let (rgbData, rgbWidth, rgbHeight) = makeJPEG(from: frame.capturedImage) else {
            return nil
        }

        let imageResolution = frame.camera.imageResolution
        let intrinsics = frame.camera.intrinsics
        let depthMap = frame.smoothedSceneDepth?.depthMap ?? frame.sceneDepth?.depthMap

        let depthWidth: Int
        let depthHeight: Int
        let depthData: Data
        let scaleX: Float
        let scaleY: Float

        if let depthMap {
            guard let encodedDepth = makeDepthData(from: depthMap) else {
                return nil
            }
            depthWidth = CVPixelBufferGetWidth(depthMap)
            depthHeight = CVPixelBufferGetHeight(depthMap)
            depthData = encodedDepth
            scaleX = Float(depthWidth) / Float(imageResolution.width)
            scaleY = Float(depthHeight) / Float(imageResolution.height)
        } else {
            depthWidth = 0
            depthHeight = 0
            depthData = Data()
            scaleX = Float(rgbWidth) / Float(imageResolution.width)
            scaleY = Float(rgbHeight) / Float(imageResolution.height)
        }

        let fx = intrinsics.columns.0.x * scaleX
        let fy = intrinsics.columns.1.y * scaleY
        let cx = intrinsics.columns.2.x * scaleX
        let cy = intrinsics.columns.2.y * scaleY

        let header = PacketHeader(
            version: 1,
            frameIndex: frameIndex,
            timestamp: frame.timestamp,
            displayOrientation: currentDisplayOrientationCode(),
            rgbWidth: rgbWidth,
            rgbHeight: rgbHeight,
            depthWidth: depthWidth,
            depthHeight: depthHeight,
            fx: fx,
            fy: fy,
            cx: cx,
            cy: cy,
            pose: poseRows(from: frame.camera.transform),
            rgbSize: rgbData.count,
            depthSize: depthData.count
        )

        let encoder = JSONEncoder()
        guard let headerData = try? encoder.encode(header) else {
            return nil
        }

        var payload = Data()
        payload.append(contentsOf: [0x4C, 0x44, 0x52, 0x31])

        var headerLength = UInt32(headerData.count).bigEndian
        withUnsafeBytes(of: &headerLength) { payload.append(contentsOf: $0) }

        payload.append(headerData)
        payload.append(rgbData)
        payload.append(depthData)
        return payload
    }

    func makeJPEG(from pixelBuffer: CVPixelBuffer) -> (Data, Int, Int)? {
        let sourceWidth = CVPixelBufferGetWidth(pixelBuffer)
        let sourceHeight = CVPixelBufferGetHeight(pixelBuffer)

        let maxDimension: CGFloat = 512
        let scale = min(1.0, maxDimension / CGFloat(max(sourceWidth, sourceHeight)))
        let targetWidth = max(1, Int(CGFloat(sourceWidth) * scale))
        let targetHeight = max(1, Int(CGFloat(sourceHeight) * scale))

        let image = CIImage(cvPixelBuffer: pixelBuffer)
        let resized = image.transformed(by: CGAffineTransform(scaleX: scale, y: scale))

        guard let cgImage = ciContext.createCGImage(resized, from: CGRect(x: 0, y: 0, width: targetWidth, height: targetHeight)) else {
            return nil
        }

        guard let data = UIImage(cgImage: cgImage).jpegData(compressionQuality: 0.45) else {
            return nil
        }

        return (data, targetWidth, targetHeight)
    }

    func makeDepthData(from depthMap: CVPixelBuffer) -> Data? {
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let pixelFormat = CVPixelBufferGetPixelFormatType(depthMap)

        var output = Data(count: width * height * MemoryLayout<UInt16>.size)

        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(depthMap, .readOnly) }

        guard let baseAddress = CVPixelBufferGetBaseAddress(depthMap) else {
            return nil
        }

        output.withUnsafeMutableBytes { destinationBytes in
            let destination = destinationBytes.bindMemory(to: UInt16.self)

            switch pixelFormat {
            case kCVPixelFormatType_DepthFloat32:
                let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float32>.size
                let source = baseAddress.assumingMemoryBound(to: Float32.self)

                for y in 0 ..< height {
                    let row = source.advanced(by: y * stride)
                    let destinationOffset = y * width
                    for x in 0 ..< width {
                        destination[destinationOffset + x] = millimeters(from: row[x])
                    }
                }

            case kCVPixelFormatType_DepthFloat16:
                let stride = CVPixelBufferGetBytesPerRow(depthMap) / MemoryLayout<Float16>.size
                let source = baseAddress.assumingMemoryBound(to: Float16.self)

                for y in 0 ..< height {
                    let row = source.advanced(by: y * stride)
                    let destinationOffset = y * width
                    for x in 0 ..< width {
                        destination[destinationOffset + x] = millimeters(from: Float(row[x]))
                    }
                }

            default:
                break
            }
        }

        return output
    }

    func currentDisplayOrientationCode() -> String {
        switch UIDevice.current.orientation {
        case .portrait:
            return "portrait"
        case .portraitUpsideDown:
            return "portraitUpsideDown"
        case .landscapeLeft:
            return "landscapeLeft"
        case .landscapeRight:
            return "landscapeRight"
        default:
            return "unknown"
        }
    }

    func millimeters(from meters: Float) -> UInt16 {
        guard meters.isFinite, meters > 0.05, meters < 12 else {
            return 0
        }

        let value = Int((meters * 1000).rounded())
        return UInt16(clamping: value)
    }

    func poseRows(from transform: simd_float4x4) -> [Float] {
        [
            transform.columns.0.x, transform.columns.1.x, transform.columns.2.x, transform.columns.3.x,
            transform.columns.0.y, transform.columns.1.y, transform.columns.2.y, transform.columns.3.y,
            transform.columns.0.z, transform.columns.1.z, transform.columns.2.z, transform.columns.3.z,
            transform.columns.0.w, transform.columns.1.w, transform.columns.2.w, transform.columns.3.w,
        ]
    }
}
