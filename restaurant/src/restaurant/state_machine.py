from smach import StateMachine

from states import LocateTable, MoveToTable, LookAtTable, GetDetections3D, MoveToDiningArea, CheckForPerson


class Tables(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "MOVE_TO_DINING_AREA",
                MoveToDiningArea(),
                transitions={"success": "LOCATE_TABLE"}
            )
            self.add(
                "LOCATE_TABLE",
                LocateTable(),
                transitions={"success": "LOOK_AT_TABLE", "end": "success"}
            )
            self.add(
                "LOOK_AT_TABLE",
                LookAtTable(),
                transitions={"success": "CHECK_FOR_PERSON"}
            )
            self.add(
                "CHECK_FOR_PERSON",
                CheckForPerson(),
                transitions={"person": "MOVE_TO_TABLE", "no_person": "LOCATE_TABLE"}
            )
            self.add(
                "MOVE_TO_TABLE",
                MoveToTable(),
                transitions={"success": "IDENTIFY_ITEMS"}
            )
            self.add(
                "IDENTIFY_ITEMS",
                GetDetections3D(),
                transitions={"success": "LOCATE_TABLE"}
            )
