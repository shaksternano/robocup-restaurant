from smach import StateMachine

from states.identify_items import IdentifyItems
from states.locate_table import LocateTable
from states.move_to_dining_area import MoveToDiningArea
from states.move_to_table import MoveToTable
from states.take_order import TakeOrder
from states.confirm_order import ConfirmOrder
from tiago_controller import TiagoController


class Tables(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"])
        with self:
            controller = TiagoController()
            self.add(
                "MOVE_TO_DINING_AREA",
                MoveToDiningArea(controller),
                transitions={"success": "LOCATE_TABLE"},
            )
            self.add(
                "LOCATE_TABLE",
                LocateTable(controller),
                transitions={"success": "MOVE_TO_TABLE", "end": "success"},
            )
            self.add(
                "MOVE_TO_TABLE",
                MoveToTable(controller),
                transitions={"success": "TAKE_ORDER"},
            )
            self.add(
                "TAKE_ORDER",
                TakeOrder(),
                transitions={"success": "CONFIRM_ORDER"},
            )
            self.add(
                "CONFIRM_ORDER",
                ConfirmOrder(),
            )
