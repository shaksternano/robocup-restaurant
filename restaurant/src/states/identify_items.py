from smach import State, UserData

from tiago_controller import TiagoController


class IdentifyItems(State):

    def __init__(self, controller: TiagoController):
        super().__init__(outcomes=["success"])
        self.controller: TiagoController = controller

    def execute(self, userdata: UserData) -> str:
        detections = self.controller.get_detections()
        print("Detections:")
        print(detections)
        return "success"
