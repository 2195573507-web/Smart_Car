import SwiftUI

public struct DirectionPadView: View {
    let send: (Direction) -> Void
    public var body: some View { VStack { Button("Forward") { send(.forward) }; HStack { Button("Left") { send(.left) }; Button("Right") { send(.right) } }; Button("Reverse") { send(.reverse) } } }
}
