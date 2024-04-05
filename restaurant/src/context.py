class Context:

    def __init__(
        self,
        mic_input: bool,
        dining_area_x: float,
        dining_area_y: float,
        kitchen_area_x: float,
        kitchen_area_y: float,
    ):
        self.mic_input: bool = mic_input

        self.dining_area_x: float = dining_area_x
        self.dining_area_y: float = dining_area_y

        self.kitchen_area_x: float = kitchen_area_x
        self.kitchen_area_y: float = kitchen_area_y

        self.customer_x: float = 0
        self.customer_y: float = 0
        self.order: str = ""
