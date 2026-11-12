"""Read-only progress-based goal selection, independent of ROS and inference."""
import math


class ProgressSubgoalSelector:
    def __init__(self, motion, final_node, mode='nearest', lookahead=1,
                 threshold=0.5, confirmations=2, min_local_mass=0.8,
                 final_tolerance=0.5):
        if mode not in ('nearest', 'passage'):
            raise ValueError('Unknown subgoal update mode')
        if mode == 'passage' and lookahead != 1:
            raise ValueError('passage requires lookahead=1')
        if not 0 <= final_node < len(motion.node_s):
            raise ValueError('Invalid final node')
        if any(not math.isfinite(v) or not 0 < v <= 1
               for v in (threshold, min_local_mass)):
            raise ValueError('Probability thresholds must be in (0, 1]')
        if not isinstance(confirmations, int) or confirmations < 1:
            raise ValueError('confirmations must be a positive integer')
        if not math.isfinite(final_tolerance) or final_tolerance < 0:
            raise ValueError('final_tolerance must be finite and nonnegative')
        self.motion, self.final_node = motion, final_node
        self.mode, self.lookahead = mode, lookahead
        self.threshold, self.confirmations = threshold, confirmations
        self.min_local_mass, self.final_tolerance = min_local_mass, final_tolerance
        self.subgoal = None
        self.reached = False
        self.last_stamp = None
        self.count = 0
        self.status = 'INITIAL'
        self.probabilities = {}

    def reset_confirmation(self):
        self.count = 0

    def update(self, closest_node, stamp_ns):
        if self.reached or stamp_ns == self.last_stamp:
            return self.subgoal, self.reached
        if self.last_stamp is not None and stamp_ns < self.last_stamp:
            self.reset_confirmation()
        self.last_stamp = stamp_ns
        initial = self.subgoal is None
        if initial or self.mode == 'nearest':
            candidate = min(closest_node + self.lookahead, self.final_node)
            if candidate != self.subgoal:
                self.reset_confirmation()
            self.subgoal = candidate
        if self.mode == 'nearest':
            # Preserve the existing mode, including its final-node decision.
            self.reached = closest_node == self.final_node
            self.status = 'GOAL_REACHED' if self.reached else 'NEAREST'
            return self.subgoal, self.reached
        self.probabilities = self.motion.progress_probabilities(
            self.subgoal, self.final_tolerance)
        is_final = self.subgoal == self.final_node
        # Preserve the immediate initial goal, including a broad global prior.
        if initial and not is_final:
            self.status = 'INITIAL'
            return self.subgoal, False
        # Keep the conservative neighborhood check for terminal arrival only.
        # Intermediate goals may already be behind after a short image gap.
        if is_final and self.probabilities['local'] < self.min_local_mass:
            self.reset_confirmation()
            self.status = 'REMOTE_BELIEF'
        elif self.probabilities['near' if is_final else 'passed'] < self.threshold:
            self.reset_confirmation()
            self.status = 'WAITING'
        else:
            self.count += 1
            self.status = 'CONFIRMING'
            if self.count >= self.confirmations:
                self.reset_confirmation()
                if is_final:
                    self.reached = True
                    self.status = 'GOAL_REACHED'
                else:
                    self.subgoal += 1
                    self.status = 'ADVANCED'
        # Check the output candidate, including a just-advanced goal. This also
        # prevents an old posterior from immediately undoing a backward rescue.
        lower = min(closest_node, self.final_node)
        upper = min(closest_node + self.lookahead, self.final_node)
        if not self.reached and not lower <= self.subgoal <= upper:
            self.subgoal = upper
            self.reset_confirmation()
            self.status = 'RECOVERED'
        return self.subgoal, self.reached
