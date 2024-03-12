from smach import StateMachine

from states.identify_items import IdentifyItems
from states.locate_table import LocateTable
from states.move_to_dining_area import MoveToDiningArea
from states.move_to_table import MoveToTable
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
                transitions={"success": "IDENTIFY_ITEMS"},
            )
            self.add(
                "IDENTIFY_ITEMS",
                IdentifyItems(controller),
            )
