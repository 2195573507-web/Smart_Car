# Findings

- The worktree is already dirty, including the three target source files.
- The calibration README still documents a 30-second vibration capture window;
  live source determines the implementation changes, while documentation scope
  will be assessed after source inspection.
- The requested binary contracts are legacy `0x0202` at 11 bytes and lifecycle
  `0x0208` at 16 bytes. Only field values may change.
