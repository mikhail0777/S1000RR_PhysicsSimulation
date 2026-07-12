import { NoAssistController } from "./base.js";
import { ThresholdController } from "./threshold.js";
import { DelayAwareController, ReductionOnlyPidController } from "./pid.js";

export function createController(name, options = {}) {
  switch (name) {
    case "threshold": return new ThresholdController(options);
    case "pid": return new ReductionOnlyPidController(options);
    case "delay-aware": return new DelayAwareController(options);
    case "none": return new NoAssistController(options);
    default: throw new Error(`Unknown controller: ${name}`);
  }
}

export { NoAssistController, ThresholdController, ReductionOnlyPidController, DelayAwareController };

