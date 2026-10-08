import { decode } from "@msgpack/msgpack";
import type { StateFrame } from "./types";

export interface StreamHandlers {
  onState: (frame: StateFrame) => void;
  onConnection: (connected: boolean) => void;
}

/** Connect to the MessagePack state stream with automatic reconnect (backoff up to 5 s). */
export function connectStream(handlers: StreamHandlers): () => void {
  let ws: WebSocket | null = null;
  let closed = false;
  let delay = 250;
  let timer: ReturnType<typeof setTimeout> | undefined;

  const open = () => {
    const proto = location.protocol === "https:" ? "wss" : "ws";
    ws = new WebSocket(`${proto}://${location.host}/api/stream`);
    ws.binaryType = "arraybuffer";
    ws.onopen = () => {
      delay = 250;
      handlers.onConnection(true);
    };
    ws.onmessage = (ev) => {
      const msg = decode(new Uint8Array(ev.data as ArrayBuffer)) as { type?: string };
      if (msg.type === "state") handlers.onState(msg as StateFrame);
    };
    ws.onclose = () => {
      handlers.onConnection(false);
      if (!closed) {
        timer = setTimeout(open, delay);
        delay = Math.min(delay * 2, 5000);
      }
    };
  };
  open();

  return () => {
    closed = true;
    clearTimeout(timer);
    ws?.close();
  };
}
