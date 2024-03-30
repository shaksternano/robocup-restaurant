from typing import Optional, Callable

from smach import StateMachine, State, UserData

from lasr_skills import Say


class Speak(StateMachine):

    def __init__(self, text: str = "", text_supplier: Optional[Callable[[], str]] = None):
        super().__init__(outcomes=["success"])
        with self:
            self.add(
                "PREPARE_SPEECH",
                Speak.PrepareSpeech(text, text_supplier),
                transitions={"success": "SAY"},
            )
            self.add(
                "SAY",
                Say(),
                transitions={
                    "succeeded": "success",
                    "aborted": "success",
                    "preempted": "success",
                },
            )

    class PrepareSpeech(State):

        def __init__(self, text: str, text_supplier: Optional[Callable[[], str]]):
            super().__init__(
                outcomes=["success"],
                output_keys=["text"],
            )
            self.text: str = text
            self.text_supplier: Optional[Callable[[], str]] = text_supplier

        def execute(self, userdata: UserData) -> str:
            if self.text_supplier is not None:
                userdata["text"] = self.text_supplier()
            else:
                userdata["text"] = self.text
            return "success"
