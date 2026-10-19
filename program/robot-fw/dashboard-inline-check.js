
  const byId = id => document.getElementById(id);
  const views = ["loading", "empty", "error", "instrumentView"];
  const feedUrl = byId("feedUrl");
  let demoMode = false;
  let feedImage = null;
  try {
    feedUrl.value = localStorage.getItem("robot-dashboard-feed-url") || "";
  } catch (error) {
    feedUrl.value = "";
  }
  if (feedUrl.value) connectFeed();

  function connectFeed() {
    const raw = feedUrl.value.trim();
    if (!raw) {
      clearFeed("Enter a camera stream URL to connect.");
      return;
    }
    let url;
    try {
      url = new URL(raw);
      if (!(url.protocol === "http:" || url.protocol === "https:")) throw new Error("URL must use HTTP or HTTPS.");
    } catch (error) {
      clearFeed(error.message || "Enter a valid HTTP(S) MJPEG URL.");
      return;
    }
    try { localStorage.setItem("robot-dashboard-feed-url", url.href); } catch (error) { /* storage may be disabled */ }
    const image = document.createElement("img");
    image.alt = "Live camera stream";
    image.addEventListener("load", () => { byId("videoStatus").textContent = "Connected to camera stream"; });
    image.addEventListener("error", () => {
      if (feedImage === image) {
        feedImage = null;
        byId("videoStatus").textContent = "Feed unavailable. Check the stream URL, network and browser cross-origin access.";
        byId("videoFrame").replaceChildren(Object.assign(document.createElement("div"), { className: "video-empty", textContent: "Could not load this camera feed." }));
      }
    });
    feedImage = image;
    byId("videoStatus").textContent = "Connecting to camera stream";
    byId("videoFrame").replaceChildren(image);
    image.src = url.href;
  }

  function clearFeed(message = "Feed disconnected") {
    feedImage = null;
    byId("videoStatus").textContent = message;
    byId("videoFrame").innerHTML = "";
    const placeholder = document.createElement("div");
    placeholder.className = "video-empty";
    const text = document.createElement("p");
    text.textContent = message;
    placeholder.append(text);
    byId("videoFrame").append(placeholder);
  }

  async function currentStatus() {
    const response = await fetch(demoMode ? "/api/demo" : "/api/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    return response.json();
  }

  function setDemo(enabled) {
    demoMode = enabled;
    byId("demoBadge").hidden = !enabled;
    connectionState = "loading";
    hasLoaded = false;
    refresh();
  }

  byId("demoMode").addEventListener("change", event => setDemo(event.target.checked));
  byId("connectFeed").addEventListener("click", connectFeed);
  byId("clearFeed").addEventListener("click", () => {
    feedUrl.value = "";
    try { localStorage.removeItem("robot-dashboard-feed-url"); } catch (error) { /* storage may be disabled */ }
    clearFeed("Feed disconnected");
  });
  feedUrl.addEventListener("keydown", event => { if (event.key === "Enter") connectFeed(); });

  let hasLoaded = false;
  let busy = false;
  let connectionState = "loading";

  function show(view) {
    for (const id of views) byId(id).hidden = id !== view;
  }

  function fixed(value, digits = 3) {
    return Number.isFinite(value) ? value.toFixed(digits) : "N/A";
  }

  function render(data) {
    byId("demoBadge").hidden = !data.demo;
    if (!data.available) {
      show("empty");
      byId("connection").dataset.kind = "offline";
      if (connectionState !== "empty") byId("connection").textContent = data.error ? "Run log marker invalid" : "No telemetry row yet";
      connectionState = "empty";
      if (data.error) byId("empty").querySelector("p").textContent = `${data.error}. Check the runtime output directory.`;
      return;
    }
    byId("empty").querySelector("p").textContent = "Start the robot runtime or point the dashboard at its configured output directory.";
    show("instrumentView");
    const stale = !data.fresh;
    const nextState = data.demo ? "demo" : stale ? "stale" : "live";
    byId("connection").dataset.kind = data.demo ? "stale" : nextState;
    if (connectionState !== nextState) {
      byId("connection").textContent = data.demo ? "Simulated preview" : stale ? "Log is stale" : "Log is updating";
    }
    connectionState = nextState;
    byId("stateTitle").textContent = data.state || "Unknown state";
    byId("stateReason").textContent = data.reason || "";
    byId("speed").textContent = fixed(data.speed_mps, 2);
    byId("steering").textContent = fixed(data.steering_rad, 3);
    byId("steeringMeter").value = Number.isFinite(data.steering_rad) ? Math.min(.55, Math.abs(data.steering_rad)) : 0;
    byId("commandReason").textContent = data.command_reason || "Command reason unavailable";
    byId("confidence").textContent = Number.isFinite(data.confidence) ? `${Math.round(data.confidence * 100)}%` : "N/A";
    byId("laneError").textContent = `${fixed(data.lane_error_m)} m`;
    byId("headingError").textContent = `${fixed(data.heading_error_rad)} rad`;
    byId("curvature").textContent = `${fixed(data.curvature_inv_m)} 1/m`;
    byId("runName").textContent = data.run || "Unknown run";
    byId("lastUpdate").textContent = data.demo ? "Synthetic sample values; no robot logs changed." : stale
      ? `Last log write ${fixed(data.age_s, 1)} seconds ago. Runtime may have stopped.`
      : "Telemetry log is updating.";
    const list = byId("events");
    list.replaceChildren();
    if (!data.events || data.events.length === 0) {
      const item = document.createElement("li");
      item.textContent = "No runtime events recorded.";
      list.append(item);
    } else {
      for (const event of [...data.events].reverse()) {
        const item = document.createElement("li");
        const name = document.createElement("span");
        name.textContent = event.event;
        const kind = document.createElement("span");
        kind.textContent = data.demo ? "simulated" : "event";
        item.append(name, kind);
        list.append(item);
      }
    }
  }

  async function refresh() {
    if (busy) return;
    busy = true;
    byId("refresh").disabled = true;
    if (!hasLoaded) show("loading");
    try {
      render(await currentStatus());
      hasLoaded = true;
    } catch (error) {
      show("error");
      byId("connection").dataset.kind = "offline";
      if (connectionState !== "error") byId("connection").textContent = "Dashboard disconnected";
      connectionState = "error";
      byId("errorText").textContent = `${error.message}. Check the Jetson process and log directory, then refresh.`;
    } finally {
      busy = false;
      byId("refresh").disabled = false;
    }
  }

  byId("refresh").addEventListener("click", refresh);
  refresh();
  window.setInterval(refresh, 1500);
