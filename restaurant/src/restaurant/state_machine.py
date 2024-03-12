from smach import StateMachine

from states.check_for_person import CheckForPerson
from states.identify_items import IdentifyItems
from states.locate_table import LocateTable
from states.look_at_table import LookAtTable
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
                MoveToDiningArea(controller, 1.4, -4.5),
                transitions={"success": "LOCATE_TABLE"},
            )
            self.add(
                "LOCATE_TABLE",
                LocateTable(controller),
                transitions={"success": "LOOK_AT_TABLE", "end": "success"},
            )
            self.add(
                "LOOK_AT_TABLE",
                LookAtTable(controller),
                transitions={"success": "CHECK_FOR_PERSON"},
            )
            self.add(
                "CHECK_FOR_PERSON",
                CheckForPerson(controller),
                transitions={"person": "MOVE_TO_TABLE", "no_person": "LOCATE_TABLE"},
            )
            self.add(
                "MOVE_TO_TABLE",
                MoveToTable(controller),
                transitions={"success": "IDENTIFY_ITEMS"},
            )
            self.add(
                "IDENTIFY_ITEMS",
                IdentifyItems(controller),
                transitions={"success": "LOCATE_TABLE"},
            )
