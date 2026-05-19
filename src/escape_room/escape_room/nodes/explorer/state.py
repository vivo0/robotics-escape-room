"""Mission FSM state enumeration."""

from enum import Enum


class State(Enum):
    EXPLORE = "explore"
    GO_TO_KEY = "go_to_key"
    PICKUP_OPEN = "pickup_open"
    PICKUP_ALIGN = "pickup_align"
    PICKUP_CLOSE = "pickup_close"
    GO_TO_PLATE = "go_to_plate"
    DROP_OPEN = "drop_open"
    GO_TO_DOOR = "go_to_door"
    EXIT_DRIVE = "exit_drive"
    DONE = "done"

    def __str__(self) -> str:
        return self.value
