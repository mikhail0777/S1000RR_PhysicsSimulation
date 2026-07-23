const requiredAliases = Object.freeze({
  time: ["time_s", "time", "timestamp_s"],
  rpm: ["engine_rpm", "rpm"],
  throttle: ["throttle_pct", "throttle_percent", "rider_throttle_pct"],
  rearSpeed: ["rear_wheel_speed_mps", "rear_speed_mps"],
  frontSpeed: ["front_wheel_speed_mps", "front_speed_mps"],
  pitch: ["pitch_deg", "pitch_rad"]
});

function splitCsvLine(line) {
  const fields = []; let field = "", quoted = false;
  for (let index = 0; index < line.length; index += 1) {
    const char = line[index];
    if (char === '"' && line[index + 1] === '"') { field += '"'; index += 1; }
    else if (char === '"') quoted = !quoted;
    else if (char === "," && !quoted) { fields.push(field.trim()); field = ""; }
    else field += char;
  }
  fields.push(field.trim()); return fields;
}

export function parseTelemetryCsv(text) {
  const lines = text.replace(/^\uFEFF/, "").split(/\r?\n/).filter(line => line.trim());
  if (lines.length < 2) throw new Error("Telemetry CSV needs a header and at least one data row.");
  const headers = splitCsvLine(lines[0]).map(value => value.toLowerCase());
  const columns = Object.fromEntries(Object.entries(requiredAliases).map(([name, aliases]) =>
    [name, aliases.map(alias => headers.indexOf(alias)).find(index => index >= 0)]));
  if (columns.time === undefined) throw new Error("Missing time column. Use time_s, time, or timestamp_s.");
  const rows = lines.slice(1).map((line, rowIndex) => {
    const values = splitCsvLine(line);
    const row = { sourceRow: rowIndex + 2 };
    for (const [name, index] of Object.entries(columns)) {
      if (index === undefined || values[index] === "") { row[name] = null; continue; }
      const value = Number(values[index]);
      if (!Number.isFinite(value)) throw new Error(`Non-numeric ${headers[index]} at row ${rowIndex + 2}.`);
      row[name] = name === "pitch" && headers[index].endsWith("_deg") ? value * Math.PI / 180 : value;
    }
    if (row.throttle !== null) row.throttle /= 100;
    return row;
  });
  for (let index = 1; index < rows.length; index += 1) {
    if (rows[index].time <= rows[index - 1].time) throw new Error(`Timestamps must increase strictly (row ${rows[index].sourceRow}).`);
  }
  return {
    rows,
    channels: Object.fromEntries(Object.entries(columns).filter(([, index]) => index !== undefined).map(([name]) => [name, true])),
    missingChannels: Object.keys(columns).filter(name => columns[name] === undefined),
    duration: rows.at(-1).time - rows[0].time
  };
}

export function compareTelemetry(measured, simulated) {
  const available = ["rpm", "pitch", "throttle"].filter(channel => measured.channels[channel]);
  const mapping = { rpm: "engineRpm", pitch: "pitch", throttle: "throttle" };
  const errors = {};
  for (const channel of available) {
    const pairs = measured.rows.map(row => {
      const nearest = simulated.reduce((best, sample) => Math.abs(sample.time - row.time) < Math.abs(best.time - row.time) ? sample : best, simulated[0]);
      return row[channel] === null ? null : nearest[mapping[channel]] - row[channel];
    }).filter(value => value !== null);
    errors[channel] = { rmse: Math.sqrt(pairs.reduce((sum, value) => sum + value * value, 0) / Math.max(1, pairs.length)), samples: pairs.length };
  }
  return errors;
}

