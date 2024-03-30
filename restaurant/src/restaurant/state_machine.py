from smach import StateMachine

from context import Context
from states.confirm_order import ConfirmOrder
from states.deliver_order import DeliverOrder
from states.locate_customer import LocateCustomer
from states.locate_staff import LocateStaff
from states.move_to_dining_area import MoveToDiningArea
from states.move_to_kitchen import MoveToKitchen
from states.move_to_point import MoveToPoint
from states.speak import Speak
from states.take_order import TakeOrder
from states.wait_for_order import WaitForOrder
from tiago_controller import TiagoController


class Tables(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"])
        with self:
            controller = TiagoController()
            context = Context()
            self.add(
                "MOVE_TO_DINING_AREA",
                MoveToDiningArea(controller),
                transitions={"success": "LOCATE_TABLE"},
            )
            self.add(
                "LOCATE_TABLE",
                LocateCustomer(controller, context),
                transitions={"success": "MOVE_TO_TABLE", "end": "success"},
            )
            self.add(
                "MOVE_TO_TABLE",
                MoveToPoint(controller),
                transitions={"success": "TAKE_ORDER"},
            )
            self.add(
                "TAKE_ORDER",
                TakeOrder(context),
                transitions={"success": "CONFIRM_ORDER"},
            )
            self.add(
                "CONFIRM_ORDER",
                ConfirmOrder(context),
                transitions={"success": "MOVE_TO_KITCHEN"},
            )
            self.add(
                "MOVE_TO_KITCHEN",
                MoveToKitchen(controller),
                transitions={"success": "LOCATE_STAFF"},
            )
            self.add(
                "LOCATE_STAFF",
                LocateStaff(controller),
                transitions={"success": "MOVE_TO_STAFF"},
            )
            self.add(
                "MOVE_TO_STAFF",
                MoveToPoint(controller),
                transitions={"success": "REQUEST_ORDER"},
            )
            self.add(
                "REQUEST_ORDER",
                Speak(text_supplier=lambda: f"Please get me {context.order}"),
                transitions={"success": "WAIT_FOR_ORDER"},
            )
            self.add(
                "WAIT_FOR_ORDER",
                WaitForOrder(),
                transitions={"success": "DELIVER_ORDER"},
            )
            self.add(
                "DELIVER_ORDER",
                DeliverOrder(controller, context),
            )
