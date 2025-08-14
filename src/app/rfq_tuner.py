import time
import logging
from enum import Enum
from typing import Optional


class TunerState(Enum):
    IDLE = "idle"
    SEEKING = "seeking_direction"
    TRACKING = "tracking"


class RFQ:
    """
    RF Cavity Q-control using reflection coefficient feedback.
    Implements a seek-and-track algorithm for tuner positioning.
    """

    def __init__(
        self,
        epics_adapter,
        motor_controller,
        coef_min: float = 0.01,
        coef_max: float = 0.05,
        test_step_mm: float = 0.1,
        tracking_step_mm: float = 0.1,
        settling_time: float = 0.2,
    ):
        """
        Args:
            epics_adapter: EPICS channel access adapter
            motor_controller: Tuner motor control interface
            coef_min: Lower threshold for reflection coefficient
            coef_max: Upper threshold triggering tuning action
            test_step_mm: Step size for direction finding [mm]
            tracking_step_mm: Step size for convergence [mm]
            settling_time: Wait time after motor move [seconds]
        """
        self.epics_ca = epics_adapter
        self.motors = motor_controller
        self.coef_min = coef_min
        self.coef_max = coef_max
        self.test_step = test_step_mm
        self.tracking_step = tracking_step_mm
        self.settling_time = settling_time

        self.state = TunerState.IDLE
        self.ref_coeff_pv_name = "PrtAcc:Pwr:ReflectionCoefCalc"

        # Logging
        self.logger = logging.getLogger(__name__)

    def read_coefficient(self) -> float:
        """Read reflection coefficient from EPICS PV."""
        return self.epics_ca.caget(self.ref_coeff_pv_name)

    def move_tuner(self, distance_mm: float) -> None:
        """
        Move tuner by specified distance.
        Positive = insert (increase frequency)
        Negative = retract (decrease frequency)
        """
        self.motors.move_relative(distance_mm)
        time.sleep(self.settling_time)

    def find_direction(self) -> int:
        """
        Determine which direction reduces reflection coefficient.
        Returns: +1 for positive direction, -1 for negative
        """
        self.logger.info("Finding optimal tuning direction...")

        # Record initial state
        initial_coef = self.read_coefficient()
        initial_position = self.motors.get_position()

        # Test positive direction
        self.move_tuner(self.test_step)
        time.sleep(self.settling_time)
        coef_positive = self.read_coefficient()

        if coef_positive < initial_coef:
            # Positive direction reduces reflection
            self.logger.info(
                f"Direction found: positive (coef: {initial_coef:.4f} → {coef_positive:.4f})"
            )
            return 1
        else:
            # Test negative direction
            # Move back to origin and then negative
            self.move_tuner(-2 * self.test_step)
            coef_negative = self.read_coefficient()

            if coef_negative < initial_coef:
                self.logger.info(
                    f"Direction found: negative (coef: {initial_coef:.4f} → {coef_negative:.4f})"
                )
                return -1
            else:
                self.logger.warning(
                    "No improvement in either direction - possible local minimum"
                )
                return 0

    def track_to_target(self, direction: int) -> bool:
        """
        Move tuner incrementally in given direction until coefficient <= coef_min.
        Returns: True if target reached, False if coefficient started increasing
        """
        self.logger.info(f"Tracking to target (direction: {direction})")

        previous_coef = self.read_coefficient()
        steps_taken = 0
        max_steps = 100

        while steps_taken < max_steps:
            # Move one tracking step
            self.move_tuner(direction * self.tracking_step)
            current_coef = self.read_coefficient()
            steps_taken += 1

            self.logger.debug(f"Step {steps_taken}: coef = {current_coef:.4f}")

            # Check if we reached target
            if current_coef <= self.coef_min:
                self.logger.info(f"Target reached: coef = {current_coef:.4f}")
                return True

            # Check if we're going wrong direction
            if current_coef > previous_coef * 1.05:  # 5% tolerance for noise
                self.logger.warning(
                    f"Coefficient increasing - wrong direction detected"
                )
                # Back off one step
                self.move_tuner(-direction * self.tracking_step)
                return False

            previous_coef = current_coef

        self.logger.error(f"Maximum steps ({max_steps}) reached without convergence")
        return False

    def run_once(self) -> None:
        """Execute one control cycle."""
        coefficient = self.read_coefficient()

        if self.coef_min <= coefficient <= self.coef_max:
            # Within acceptable range
            if self.state != TunerState.IDLE:
                self.logger.info(
                    f"Coefficient in range ({coefficient:.4f}), returning to IDLE"
                )
                self.state = TunerState.IDLE
            return

        if coefficient > self.coef_max:
            self.logger.info(
                f"Coefficient too high: {coefficient:.4f} > {self.coef_max:.4f}"
            )

            # Find direction
            self.state = TunerState.SEEKING
            direction = self.find_direction()

            if direction == 0:
                # Could not determine direction
                self.logger.error("Cannot determine tuning direction")
                self.state = TunerState.IDLE
                return

            # Track to target
            self.state = TunerState.TRACKING
            success = self.track_to_target(direction)

            if not success:
                # Tracking failed, will retry on next cycle
                self.logger.warning("Tracking failed, will retry")

    def run_continuous(self, update_rate: float = 1.0) -> None:
        """
        Run control loop continuously.
        Args:
            update_rate: Loop frequency in Hz
        """
        self.logger.info(f"Starting continuous control loop at {update_rate} Hz")

        try:
            while True:
                self.run_once()
                time.sleep(1.0 / update_rate)

        except KeyboardInterrupt:
            self.logger.info("Control loop stopped by user")
        except Exception as e:
            self.logger.error(f"Control loop error: {e}")
            raise


# Usage example:
if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(level=logging.INFO)

    # Initialize adapters (these would be your actual EPICS and motor interfaces)
    from epics import caget, caput

    class EpicsAdapter:
        def caget(self, pv_name):
            return caget(pv_name)

    class MotorController:
        def __init__(self, motor_pv_base):
            self.pv_base = motor_pv_base

        def move_relative(self, distance_mm):
            caput(f"{self.pv_base}:REL", distance_mm)

        def move_absolute(self, position_mm):
            caput(f"{self.pv_base}:ABS", position_mm)

        def get_position(self):
            return caget(f"{self.pv_base}:POS")

    # Create and run controller
    epics_adapter = EpicsAdapter()
    motor = MotorController("PrtAcc:Tuner1")

    rfq = RFQ(epics_adapter, motor, coef_min=0.01, coef_max=0.05)
    rfq.run_continuous(update_rate=2.0)  # 2 Hz control loop