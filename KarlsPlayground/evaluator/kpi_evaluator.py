
from typing import Any, Sequence



class KPIEvaluator:
    """Evaluator for parking maneuver Key Performance Indicators (KPIs)."""

    @staticmethod
    def direction_changes(path: Sequence[Any]) -> int:
        """Count the number of direction changes in the planned path."""
        return sum(1 for i in range(1, len(path)) if path[i].direction != path[i - 1].direction)

    @staticmethod
    def maneuver_blocks(path: Sequence[Any]) -> int:
        """Count the number of maneuver blocks (continuous segments in one direction)."""
        if not path:
            return 0
        return KPIEvaluator.direction_changes(path) + 1
    
    