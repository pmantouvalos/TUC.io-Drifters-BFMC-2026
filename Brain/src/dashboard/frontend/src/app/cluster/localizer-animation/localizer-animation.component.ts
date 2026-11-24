import { Component, ElementRef, ViewChild, OnInit, OnDestroy } from '@angular/core';
import { Subscription } from 'rxjs';
import { WebSocketService } from '../../webSocket/web-socket.service';

const MAP_W = 20.67;
const MAP_H = 13.76;

// Canvas pixel dimensions (same aspect ratio as the map)
const CANVAS_W = 414;
const CANVAS_H = 276;

const MODE_COLORS: Record<string, string> = {
  GLOBAL: '#facc15',   // yellow
  LOCAL: '#4ade80',    // green
  RECOVER: '#fb923c',  // orange
};

@Component({
  selector: 'app-localizer-animation',
  standalone: true,
  templateUrl: './localizer-animation.component.html',
  styleUrl: './localizer-animation.component.css',
})
export class LocalizerAnimationComponent implements OnInit, OnDestroy {
  @ViewChild('canvas', { static: true }) canvasRef!: ElementRef<HTMLCanvasElement>;

  public mode: string = '—';
  private ctx!: CanvasRenderingContext2D;
  private sub: Subscription | undefined;

  // Track image
  private trackImg = new Image();
  private trackLoaded = false;

  constructor(private wsService: WebSocketService) {}

  ngOnInit(): void {
    const canvas = this.canvasRef.nativeElement;
    canvas.width = CANVAS_W;
    canvas.height = CANVAS_H;
    this.ctx = canvas.getContext('2d')!;

    this.trackImg.src = 'assets/Track.svg';
    this.trackImg.onload = () => {
      this.trackLoaded = true;
      this.drawEmpty();
    };

    this.drawEmpty();

    this.sub = this.wsService.receiveLiveAnimation().subscribe((msg) => {
      const data = msg.value;
      this.mode = data.mode ?? '—';
      this.draw(data);
    });
  }

  ngOnDestroy(): void {
    this.sub?.unsubscribe();
  }

  private toCanvas(mx: number, my: number): [number, number] {
    return [
      (mx / MAP_W) * CANVAS_W,
      (1 - my / MAP_H) * CANVAS_H,
    ];
  }

  private drawEmpty(): void {
    const ctx = this.ctx;

    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);

    if (this.trackLoaded) {
      ctx.drawImage(this.trackImg, 0, 0, CANVAS_W, CANVAS_H);
    } else {
      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);
    }

    ctx.fillStyle = '#444';
    ctx.font = '14px monospace';
    ctx.fillText('Waiting for localizer…', 10, CANVAS_H / 2);
  }

  private draw(data: { particles: number[][], pose: number[], mode: string }): void {
    const ctx = this.ctx;

    // Background track
    ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);

    if (this.trackLoaded) {
      ctx.drawImage(this.trackImg, 0, 0, CANVAS_W, CANVAS_H);
    } else {
      ctx.fillStyle = '#111';
      ctx.fillRect(0, 0, CANVAS_W, CANVAS_H);
    }

    const particleColor = MODE_COLORS[data.mode] ?? '#fff';

    // Draw particles
    for (const [px, py, pw] of data.particles) {
      const [cx, cy] = this.toCanvas(px, py);
      const alpha = Math.min(1, 0.3 + pw * 3);

      ctx.globalAlpha = alpha;
      ctx.fillStyle = particleColor;
      ctx.beginPath();
      ctx.arc(cx, cy, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }

    ctx.globalAlpha = 1;

    // Draw pose arrow
    if (data.pose) {
      const [ex, ey, theta] = data.pose;
      const [cx, cy] = this.toCanvas(ex, ey);

      const arrowLen = 18;
      const dx = Math.cos(theta) * arrowLen;
      const dy = -Math.sin(theta) * arrowLen;

      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx + dx, cy + dy);
      ctx.stroke();

      const headLen = 7;
      const angle = Math.atan2(dy, dx);

      ctx.beginPath();
      ctx.moveTo(cx + dx, cy + dy);
      ctx.lineTo(cx + dx - headLen * Math.cos(angle - 0.4), cy + dy - headLen * Math.sin(angle - 0.4));
      ctx.lineTo(cx + dx - headLen * Math.cos(angle + 0.4), cy + dy - headLen * Math.sin(angle + 0.4));
      ctx.closePath();
      ctx.fillStyle = '#fff';
      ctx.fill();

      ctx.fillStyle = '#ef4444';
      ctx.beginPath();
      ctx.arc(cx, cy, 5, 0, Math.PI * 2);
      ctx.fill();
    }
  }
}