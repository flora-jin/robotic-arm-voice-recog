import re
from difflib import SequenceMatcher
from typing import Iterable

SIMILARITY_THRESHOLD = 0.75

AFFIRMATIVE_PHRASES = ("confirm", "yes", "yeah", "yep", "yup", "okay")
NEGATIVE_PHRASES = ("cancel", "no", "stop", "abort", "never mind")
IGNORE_PHRASES = ("um", "um...", "uh", "uh...", "hmm", "hmm...", "let me think", "you know", "like", "actually", "basically", "what", "wait", "thank you", "oh", "oh...")


def normalize_phrase(text: str) -> str:
    """Lowercase and strip punctuation/noise so near-matches compare better."""
    text = text.lower().strip()
    text = re.sub(r"[^a-z0-9\s]", " ", text)
    return " ".join(text.split())


def phrase_similarity(a: str, b: str) -> float:
    return SequenceMatcher(None, a, b).ratio()


def is_similar_command(text: str, target_phrase: str) -> bool:
    """Fuzzy-match a command by checking the full phrase and nearby word windows."""
    normalized_text = normalize_phrase(text)
    normalized_target = normalize_phrase(target_phrase)

    if not normalized_text or not normalized_target:
        return False

    if normalized_target in normalized_text:
        return True

    best_score = phrase_similarity(normalized_text, normalized_target)
    text_words = normalized_text.split()
    target_len = len(normalized_target.split())

    # Compare with windows around target size to catch partial-sentence matches.
    for size in (max(1, target_len - 1), target_len, target_len + 1):
        if size > len(text_words):
            continue
        for start in range(len(text_words) - size + 1):
            window = " ".join(text_words[start : start + size])
            best_score = max(best_score, phrase_similarity(window, normalized_target))

    return best_score >= SIMILARITY_THRESHOLD


def matches_any_command(text: str, phrases: Iterable[str]) -> bool:
    return any(is_similar_command(text, phrase) for phrase in phrases)


def has_similar_intent_prefix(command_id: str, expected_prefix: str) -> bool:
    """Fuzzy-match command prefix (e.g. CHECK_*, ASSIGN_*) to tolerate minor model typos."""
    if not command_id or not expected_prefix:
        return False

    prefix = command_id.split("_", 1)[0].strip().lower()
    expected = expected_prefix.strip().lower()
    if not prefix or not expected:
        return False

    return phrase_similarity(prefix, expected) >= SIMILARITY_THRESHOLD
