export class Controller {
  constructor(options = {}) {
    this.options = options;
    this.status = "ready";
    this.lastComputeMs = 0;
  }

  reset() {
    this.status = "ready";
    this.lastComputeMs = 0;
  }

  compute(_observation, _context) {
    return { torqueCeiling: Number.POSITIVE_INFINITY, status: "inactive" };
  }
}

export class NoAssistController extends Controller {
  compute() {
    return { torqueCeiling: Number.POSITIVE_INFINITY, status: "off" };
  }
}

export function timedCompute(controller, operation) {
  const start = performance.now();
  const result = operation();
  controller.lastComputeMs = performance.now() - start;
  return { ...result, computeMs: controller.lastComputeMs };
}

