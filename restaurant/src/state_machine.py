import rospy
from smach import StateMachine

from context import Context
from states.deliver_order import DeliverOrder
from states.locate_customer import LocateCustomer
from states.locate_staff import LocateStaff
from states.move_to_dining_area import MoveToDiningArea
from states.move_to_kitchen import MoveToKitchen
from states.move_to_point import MoveToPoint
from states.speak import Speak
from states.take_order import TakeOrder
from states.wait import Wait
from states.wait_for_order import WaitForOrder
from tiago_controller import TiagoController


class Tables(StateMachine):

    def __init__(self):
        super().__init__(outcomes=["success"])
        with self:
            controller = TiagoController()
            context = create_context()
            self.add(
                "MOVE_TO_DINING_AREA",
                MoveToDiningArea(controller, context),
                transitions={"success": "LOCATE_CUSTOMER"},
            )
            self.add(
                "LOCATE_CUSTOMER",
                LocateCustomer(controller, context),
                transitions={"success": "MOVE_TO_CUSTOMER", "failure": "WAIT_FOR_CUSTOMER"},
            )
            self.add(
                "WAIT_FOR_CUSTOMER",
                Wait(),
                transitions={"success": "LOCATE_CUSTOMER"},
            )
            self.add(
                "MOVE_TO_CUSTOMER",
                MoveToPoint(controller),
                transitions={"success": "TAKE_ORDER", "failure": "LOCATE_CUSTOMER"},
            )
            self.add(
                "TAKE_ORDER",
                TakeOrder(context),
                transitions={"success": "CONFIRM_ORDER"},
            )
            self.add(
                "CONFIRM_ORDER",
                Speak(text_supplier=lambda: f"I will bring you {context.order}"),
                transitions={"success": "MOVE_TO_KITCHEN"},
            )
            self.add(
                "MOVE_TO_KITCHEN",
                MoveToKitchen(controller, context),
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
                transitions={"success": "REQUEST_ORDER", "failure": "LOCATE_STAFF"},
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


def create_context() -> Context:
    mic_input: bool = rospy.get_param("~mic_input")
    dining_area_x: float = rospy.get_param("~dining_area_x")
    dining_area_y: float = rospy.get_param("~dining_area_y")
    kitchen_area_x: float = rospy.get_param("~kitchen_area_x")
    kitchen_area_y: float = rospy.get_param("~kitchen_area_y")
    return Context(
        mic_input,
        dining_area_x,
        dining_area_y,
        kitchen_area_x,
        kitchen_area_y,
    )
