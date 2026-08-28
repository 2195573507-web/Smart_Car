import SwiftUI

struct SceneKitPoseView: View {
    let attitude: AttitudeData
    let targetYawDegrees: Float?

    var body: some View {
        ZStack {
            Color.black.opacity(0.18)

            VehicleOrientationModel()
                .rotation3DEffect(
                    .radians(Double(attitude.pitchRad)),
                    axis: (x: 1, y: 0, z: 0)
                )
                .rotation3DEffect(
                    .radians(Double(attitude.rollRad)),
                    axis: (x: 0, y: 1, z: 0)
                )
                .rotationEffect(.radians(Double(attitude.yawRad)))
                .animation(.easeOut(duration: 0.12), value: attitude.rollRad)
                .animation(.easeOut(duration: 0.12), value: attitude.pitchRad)
                .animation(.easeOut(duration: 0.12), value: attitude.yawRad)
        }
        .frame(maxWidth: .infinity, minHeight: 190, maxHeight: 230)
        .clipShape(RoundedRectangle(cornerRadius: 12))
        .accessibilityElement(children: .ignore)
        .accessibilityLabel("车辆姿态模型")
        .accessibilityValue(
            "Roll \(degreeText(attitude.rollRad)), Pitch \(degreeText(attitude.pitchRad)), Yaw \(degreeText(attitude.yawRad))"
        )
    }
}

private struct VehicleOrientationModel: View {
    var body: some View {
        ZStack {
            RoundedRectangle(cornerRadius: 32, style: .continuous)
                .fill(.black.opacity(0.28))
                .frame(width: 126, height: 164)
                .blur(radius: 7)
                .offset(y: 5)

            HStack(spacing: 0) {
                wheelColumn
                Spacer(minLength: 0)
                wheelColumn
            }
            .frame(width: 144, height: 164)

            VehicleBodyShape()
                .fill(Color(red: 0.08, green: 0.23, blue: 0.36))
                .frame(width: 112, height: 160)
                .overlay {
                    VehicleBodyShape()
                        .stroke(Color.cyan.opacity(0.62), lineWidth: 1.5)
                }
                .overlay {
                    VehicleCabinShape()
                        .fill(Color(red: 0.13, green: 0.34, blue: 0.49).opacity(0.9))
                        .frame(width: 70, height: 91)
                        .overlay {
                            VehicleCabinShape()
                                .stroke(Color.white.opacity(0.24), lineWidth: 1)
                        }
                }
                .overlay(alignment: .top) {
                    HStack(spacing: 26) {
                        frontLamp.rotationEffect(.degrees(-18))
                        frontLamp.rotationEffect(.degrees(18))
                    }
                    .padding(.top, 10)
                }
                .overlay(alignment: .bottom) {
                    HStack(spacing: 22) {
                        tailLamp.rotationEffect(.degrees(15))
                        tailLamp.rotationEffect(.degrees(-15))
                    }
                    .padding(.bottom, 8)
                }
        }
        .frame(width: 156, height: 184)
    }

    private var frontLamp: some View {
        Capsule()
            .fill(Color(red: 1.0, green: 0.93, blue: 0.72))
            .frame(width: 22, height: 5)
            .shadow(color: Color(red: 1.0, green: 0.84, blue: 0.42).opacity(0.7), radius: 4)
    }

    private var tailLamp: some View {
        Capsule()
            .fill(Color(red: 1.0, green: 0.22, blue: 0.28))
            .frame(width: 19, height: 5)
            .shadow(color: .red.opacity(0.7), radius: 4)
    }

    private var wheelColumn: some View {
        VStack(spacing: 49) {
            wheel
            wheel
        }
    }

    private var wheel: some View {
        Capsule()
            .fill(Color(red: 0.16, green: 0.18, blue: 0.21))
            .frame(width: 14, height: 36)
            .overlay(Capsule().stroke(Color.white.opacity(0.28), lineWidth: 1))
    }
}

private struct VehicleBodyShape: Shape {
    func path(in rect: CGRect) -> Path {
        let x = rect.midX
        let top = rect.minY
        let bottom = rect.maxY
        let left = rect.minX
        let right = rect.maxX
        var path = Path()
        path.move(to: CGPoint(x: x, y: top))
        path.addQuadCurve(to: CGPoint(x: right - 12, y: top + 25), control: CGPoint(x: right - 22, y: top + 5))
        path.addQuadCurve(to: CGPoint(x: right, y: bottom - 22), control: CGPoint(x: right + 1, y: bottom - 48))
        path.addQuadCurve(to: CGPoint(x: x, y: bottom), control: CGPoint(x: right - 19, y: bottom - 1))
        path.addQuadCurve(to: CGPoint(x: left, y: bottom - 22), control: CGPoint(x: left + 19, y: bottom - 1))
        path.addQuadCurve(to: CGPoint(x: left + 12, y: top + 25), control: CGPoint(x: left - 1, y: bottom - 48))
        path.addQuadCurve(to: CGPoint(x: x, y: top), control: CGPoint(x: left + 22, y: top + 5))
        path.closeSubpath()
        return path
    }
}

private struct VehicleCabinShape: Shape {
    func path(in rect: CGRect) -> Path {
        let x = rect.midX
        var path = Path()
        path.move(to: CGPoint(x: x, y: rect.minY))
        path.addQuadCurve(to: CGPoint(x: rect.maxX, y: rect.minY + 16), control: CGPoint(x: rect.maxX - 14, y: rect.minY + 2))
        path.addLine(to: CGPoint(x: rect.maxX, y: rect.maxY - 16))
        path.addQuadCurve(to: CGPoint(x: x, y: rect.maxY), control: CGPoint(x: rect.maxX - 14, y: rect.maxY - 2))
        path.addQuadCurve(to: CGPoint(x: rect.minX, y: rect.maxY - 16), control: CGPoint(x: rect.minX + 14, y: rect.maxY - 2))
        path.addLine(to: CGPoint(x: rect.minX, y: rect.minY + 16))
        path.addQuadCurve(to: CGPoint(x: x, y: rect.minY), control: CGPoint(x: rect.minX + 14, y: rect.minY + 2))
        path.closeSubpath()
        return path
    }
}
