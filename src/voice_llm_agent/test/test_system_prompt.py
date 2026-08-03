from voice_llm_agent.domain.system_prompt import SYSTEM_PROMPT


def test_system_prompt_does_not_name_a_specific_robot() -> None:
    assert "jetrover" not in SYSTEM_PROMPT.lower()


def test_system_prompt_requires_vietnamese() -> None:
    assert "tiếng Việt" in SYSTEM_PROMPT or "Việt" in SYSTEM_PROMPT


def test_system_prompt_uses_generic_robot_identity() -> None:
    assert "robot thông minh" in SYSTEM_PROMPT
