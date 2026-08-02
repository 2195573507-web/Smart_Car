import Foundation
import Darwin

final class SerialPortService: @unchecked Sendable {
    private let queue = DispatchQueue(label: "com.smartcar.logger.serial", qos: .userInitiated)
    private var fileDescriptor: Int32 = -1
    private var source: DispatchSourceRead?
    private var dataHandler: (@Sendable (Data) -> Void)?
    private var errorHandler: (@Sendable (String) -> Void)?

    func open(
        path: String,
        onData: @escaping @Sendable (Data) -> Void,
        onError: @escaping @Sendable (String) -> Void
    ) throws {
        try queue.sync {
            closeLocked()

            let fd = Darwin.open(path, O_RDWR | O_NOCTTY | O_NONBLOCK)
            guard fd >= 0 else {
                throw SerialPortError.openFailed(path: path, reason: String(cString: strerror(errno)))
            }

            do {
                try configure(fd: fd)
            } catch {
                Darwin.close(fd)
                throw error
            }

            fileDescriptor = fd
            dataHandler = onData
            errorHandler = onError

            let readSource = DispatchSource.makeReadSource(fileDescriptor: fd, queue: queue)
            readSource.setEventHandler { [weak self] in
                self?.readAvailableBytes()
            }
            source = readSource
            readSource.resume()
        }
    }

    func close() {
        queue.sync { closeLocked() }
    }

    deinit {
        queue.sync { closeLocked() }
    }

    private func configure(fd: Int32) throws {
        var options = termios()
        guard tcgetattr(fd, &options) == 0 else {
            throw SerialPortError.configurationFailed(String(cString: strerror(errno)))
        }

        cfmakeraw(&options)
        guard cfsetspeed(&options, speed_t(B115200)) == 0 else {
            throw SerialPortError.configurationFailed(String(cString: strerror(errno)))
        }
        options.c_cflag |= tcflag_t(CLOCAL | CREAD)
        options.c_cflag &= ~tcflag_t(CSIZE | PARENB | CSTOPB | CRTSCTS)
        options.c_cflag |= tcflag_t(CS8)
        options.c_cc.0 = 0
        options.c_cc.1 = 0

        guard tcsetattr(fd, TCSANOW, &options) == 0 else {
            throw SerialPortError.configurationFailed(String(cString: strerror(errno)))
        }
    }

    private func readAvailableBytes() {
        guard fileDescriptor >= 0 else { return }
        var buffer = [UInt8](repeating: 0, count: 4096)

        while true {
            let count = Darwin.read(fileDescriptor, &buffer, buffer.count)
            if count > 0 {
                dataHandler?(Data(buffer[0..<count]))
            } else if count == 0 {
                errorHandler?("串口设备已断开")
                closeLocked()
                return
            } else if errno == EAGAIN || errno == EWOULDBLOCK {
                return
            } else if errno == EINTR {
                continue
            } else {
                errorHandler?(String(format: "串口读取失败：%@", String(cString: strerror(errno))))
                closeLocked()
                return
            }
        }
    }

    // All descriptor and dispatch-source ownership remains on `queue`.
    private func closeLocked() {
        source?.cancel()
        source = nil
        if fileDescriptor >= 0 {
            Darwin.close(fileDescriptor)
            fileDescriptor = -1
        }
        dataHandler = nil
        errorHandler = nil
    }
}

enum SerialPortError: LocalizedError {
    case openFailed(path: String, reason: String)
    case configurationFailed(String)

    var errorDescription: String? {
        switch self {
        case .openFailed(let path, let reason): "无法打开 \(path)：\(reason)"
        case .configurationFailed(let reason): "无法配置 115200 8N1：\(reason)"
        }
    }
}
