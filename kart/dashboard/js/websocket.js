export class TelemetryConnection extends EventTarget {
  constructor(store) {
    super();
    this.store = store;
    this.socket = null;
    this.retryMs = 500;
    this.closedByUser = false;
  }

  connect() {
    this.closedByUser = false;
    const protocol = location.protocol === "https:" ? "wss:" : "ws:";
    this.store.setConnection("connecting");
    this.socket = new WebSocket(`${protocol}//${location.host}/ws`);
    this.socket.addEventListener("open", () => {
      this.retryMs = 500;
      this.store.setConnection("live");
    });
    this.socket.addEventListener("message", event => {
      const message = JSON.parse(event.data);
      if (message.type === "telemetry") {
        this.store.append(message.record);
        this.dispatchEvent(new CustomEvent("telemetry", {detail: message.record}));
      }
      if (message.type === "end_of_replay") {
        this.store.setConnection("replay");
      }
    });
    this.socket.addEventListener("close", () => {
      this.store.setConnection("disconnected");
      if (!this.closedByUser) {
        setTimeout(() => this.connect(), this.retryMs);
        this.retryMs = Math.min(8000, this.retryMs * 1.7);
      }
    });
    this.socket.addEventListener("error", () => this.store.setConnection("disconnected"));
  }

  close() {
    this.closedByUser = true;
    this.socket?.close();
  }
}
