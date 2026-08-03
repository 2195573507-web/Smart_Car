import Foundation
import Darwin

struct SerialPortOpenInfo: Sendable {
    let fileDescriptor: Int32
    let readSourceActive: Bool
}

final class SerialPortService: @unchecked Sendable {
    private let queue = DispatchQueue(label: "com.smartcar.logger.serial", qos: .userInitiated)
    private var fileDescriptor: Int32 = -1
    private var source: DispatchSourceRead?
    private var pollingTimer: DispatchSourceTimer?
    private var dataHandler: (@Sendable (Data) -> Void)?
    private var diagnosticHandler: (@Sendable (String) -> Void)?
    private var errorHandler: (@Sendable (String) -> Void)?

    func open(
        path: String,
        onData: @escaping @Sendable (Data) -> Void,
        onReadCall: @escaping @Sendable () -> Void,
        onDiagnostic: @escaping @Sendable (String) -> Void,
        onError: @escaping @Sendable (String) -> Void
    ) throws -> SerialPortOpenInfo {
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
            diagnosticHandler = onDiagnostic
            errorHandler = onError

            emitDescriptorDiagnostics(fd: fd)
            emitTermiosDiagnostics(fd: fd)

            let readSource = DispatchSource.makeReadSource(fileDescriptor: fd, queue: queue)
            readSource.setEventHandler { [weak self] in
                onReadCall()
                self?.readAvailableBytes()
            }
            source = readSource
            readSource.resume()

            // Temporary diagnostic path: exercise read(2) independently of the
            // dispatch source so that missing events can be separated from an
            // empty/non-responsive serial device.
            let timer = DispatchSource.makeTimerSource(queue: queue)
            timer.schedule(deadline: .now() + .milliseconds(100), repeating: .milliseconds(100))
            timer.setEventHandler { [weak self] in
                self?.pollReadOnce()
            }
            pollingTimer = timer
            timer.resume()
            return SerialPortOpenInfo(fileDescriptor: fd, readSourceActive: true)
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
        setControlCharacter(&options, at: Int(VMIN), value: 0)
        setControlCharacter(&options, at: Int(VTIME), value: 0)

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
                // With VMIN=0/VTIME=0 and O_NONBLOCK, zero means that no byte
                // is currently available; it is not sufficient evidence of a
                // hangup. The polling path records this result explicitly.
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

    private func pollReadOnce() {
        guard fileDescriptor >= 0 else { return }
        var buffer = [UInt8](repeating: 0, count: 4096)
        let count = Darwin.read(fileDescriptor, &buffer, buffer.count)
        let readErrno: Int32 = count < 0 ? errno : 0
        let errnoText = readErrno == 0 ? "OK" : String(cString: strerror(readErrno))
        diagnosticHandler?("POLL_READ fd=\(fileDescriptor) result=\(count) errno=\(readErrno) (\(errnoText))")

        if count > 0 {
            dataHandler?(Data(buffer[0..<count]))
        } else if count < 0 && readErrno != EAGAIN && readErrno != EWOULDBLOCK {
            errorHandler?(String(format: "串口读取失败：%@", errnoText))
            closeLocked()
        }
    }

    private func emitDescriptorDiagnostics(fd: Int32) {
        let flags = fcntl(fd, F_GETFL)
        if flags < 0 {
            diagnosticHandler?("FD_STATUS fd=\(fd) fcntl_getfl=\(flags) errno=\(errno) (\(String(cString: strerror(errno))))")
        } else {
            let accessMode = flags & O_ACCMODE
            diagnosticHandler?("FD_STATUS fd=\(fd) flags=0x\(String(flags, radix: 16)) access=0x\(String(accessMode, radix: 16)) O_RDWR=\(accessMode == O_RDWR ? 1 : 0) open_args=O_RDWR|O_NOCTTY|O_NONBLOCK O_NONBLOCK=\((flags & O_NONBLOCK) != 0 ? 1 : 0)")
        }
    }

    private func emitTermiosDiagnostics(fd: Int32) {
        var options = termios()
        guard tcgetattr(fd, &options) == 0 else {
            let readErrno = errno
            diagnosticHandler?("TERMIOS fd=\(fd) tcgetattr=-1 errno=\(readErrno) (\(String(cString: strerror(readErrno))))")
            return
        }

        let vmin = controlCharacter(options, at: Int(VMIN))
        let vtime = controlCharacter(options, at: Int(VTIME))
        let cread = (options.c_cflag & tcflag_t(CREAD)) != 0 ? 1 : 0
        let clocal = (options.c_cflag & tcflag_t(CLOCAL)) != 0 ? 1 : 0
        diagnosticHandler?("TERMIOS fd=\(fd) ispeed=\(cfgetispeed(&options)) ospeed=\(cfgetospeed(&options)) cflag=0x\(String(options.c_cflag, radix: 16)) CREAD=\(cread) CLOCAL=\(clocal) VMIN=\(vmin) VTIME=\(vtime)")
    }

    private func controlCharacter(_ options: termios, at index: Int) -> cc_t {
        withUnsafePointer(to: options.c_cc) { pointer in
            pointer.withMemoryRebound(to: cc_t.self, capacity: Int(NCCS)) { values in
                values[index]
            }
        }
    }

    private func setControlCharacter(_ options: inout termios, at index: Int, value: cc_t) {
        withUnsafeMutablePointer(to: &options.c_cc) { pointer in
            pointer.withMemoryRebound(to: cc_t.self, capacity: Int(NCCS)) { values in
                values[index] = value
            }
        }
    }

    // All descriptor and dispatch-source ownership remains on `queue`.
    private func closeLocked() {
        pollingTimer?.cancel()
        pollingTimer = nil
        source?.cancel()
        source = nil
        if fileDescriptor >= 0 {
            Darwin.close(fileDescriptor)
            fileDescriptor = -1
        }
        dataHandler = nil
        diagnosticHandler = nil
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
