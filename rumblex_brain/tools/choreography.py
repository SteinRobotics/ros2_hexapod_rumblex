"""Compatibility imports for existing choreography recipes.

New recipes can import directly from sequence_generator.
"""
from sequence_generator import (  # noqa: F401
    CURVES, LEGS, MINIMUM_ABSOLUTE_POSE_DURATION,
    Sequence, Stage, beats, hold, legs, pose, positive, repeat, vector,
)
