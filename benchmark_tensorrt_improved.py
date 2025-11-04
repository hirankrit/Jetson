#!/usr/bin/env python3
"""
Improved TensorRT Benchmark Script
Compares PyTorch vs TensorRT model performance with comprehensive statistics

Features:
- Configurable via command-line arguments
- Detailed percentile statistics (p50, p95, p99)
- Error handling and logging
- GPU memory tracking
- JSON output support

Author: Generated with Claude Code
Date: 2025-11-04
"""

import argparse
import logging
import time
import json
import sys
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import torch
from ultralytics import YOLO


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


def benchmark_model(
    model_path: str,
    test_image: str,
    num_runs: int = 100,
    warmup_runs: int = 10
) -> Optional[Dict]:
    """
    Benchmark a YOLO model with comprehensive statistics

    Args:
        model_path: Path to model file (.pt or .engine)
        test_image: Path to test image
        num_runs: Number of inference runs for benchmarking
        warmup_runs: Number of warmup runs before benchmarking

    Returns:
        dict: Benchmark results, or None if error occurs
    """
    model_name = Path(model_path).name
    logger.info(f"{'='*60}")
    logger.info(f"Benchmarking: {model_name}")
    logger.info(f"{'='*60}")

    # Load model with error handling
    try:
        model = YOLO(model_path)
        logger.info(f"✅ Model loaded successfully")
    except Exception as e:
        logger.error(f"❌ Failed to load model: {e}")
        return None

    # Warmup runs
    logger.info(f"Warming up ({warmup_runs} runs)...")
    try:
        for _ in range(warmup_runs):
            _ = model(test_image, device=0, verbose=False)
    except Exception as e:
        logger.error(f"❌ Warmup failed: {e}")
        return None

    # Benchmark runs
    logger.info(f"Running benchmark ({num_runs} runs)...")
    times: List[float] = []

    # Check initial GPU memory
    if torch.cuda.is_available():
        torch.cuda.synchronize()
        mem_before = torch.cuda.memory_allocated(0) / 1024**2  # MB
    else:
        mem_before = 0
        logger.warning("⚠️  CUDA not available, GPU memory tracking disabled")

    try:
        for i in range(num_runs):
            if torch.cuda.is_available():
                torch.cuda.synchronize()

            start = time.perf_counter()
            results = model(test_image, device=0, verbose=False)

            if torch.cuda.is_available():
                torch.cuda.synchronize()

            end = time.perf_counter()
            times.append((end - start) * 1000)  # Convert to ms

            if (i + 1) % 20 == 0:
                logger.info(f"  Progress: {i+1}/{num_runs}")

    except Exception as e:
        logger.error(f"❌ Benchmark failed: {e}")
        return None

    # Check final GPU memory
    if torch.cuda.is_available():
        torch.cuda.synchronize()
        mem_after = torch.cuda.memory_allocated(0) / 1024**2  # MB
    else:
        mem_after = 0

    # Calculate comprehensive statistics
    times_array = np.array(times)

    # Get detection results for accuracy check
    try:
        final_results = model(test_image, device=0, verbose=False)
        num_detections = len(final_results[0].boxes)
    except Exception as e:
        logger.warning(f"⚠️  Could not get detection count: {e}")
        num_detections = 0

    results_dict = {
        'model_path': str(model_path),
        'model_name': model_name,
        'num_runs': num_runs,
        'warmup_runs': warmup_runs,

        # Time statistics
        'avg_time_ms': float(np.mean(times_array)),
        'std_time_ms': float(np.std(times_array)),
        'min_time_ms': float(np.min(times_array)),
        'max_time_ms': float(np.max(times_array)),
        'median_time_ms': float(np.median(times_array)),
        'p95_time_ms': float(np.percentile(times_array, 95)),
        'p99_time_ms': float(np.percentile(times_array, 99)),

        # FPS
        'fps': float(1000 / np.mean(times_array)),

        # Memory
        'gpu_memory_mb': float(mem_after - mem_before),

        # Detections
        'num_detections': int(num_detections)
    }

    logger.info(f"✅ Benchmark complete: {results_dict['avg_time_ms']:.2f}ms avg, {results_dict['fps']:.1f} FPS")

    return results_dict


def print_comparison(pytorch_results: Dict, tensorrt_results: Dict) -> None:
    """Print detailed comparison results"""

    print(f"\n{'='*90}")
    print(f"{'BENCHMARK COMPARISON RESULTS':^90}")
    print(f"{'='*90}\n")

    print(f"{'Metric':<30} {'PyTorch':<20} {'TensorRT':<20} {'Improvement':>15}")
    print(f"{'-'*30} {'-'*20} {'-'*20} {'-'*15}")

    # Inference times
    metrics = [
        ('Avg Inference Time (ms)', 'avg_time_ms', 'speedup'),
        ('Min Time (ms)', 'min_time_ms', None),
        ('Median Time (ms)', 'median_time_ms', None),
        ('P95 Time (ms)', 'p95_time_ms', None),
        ('P99 Time (ms)', 'p99_time_ms', None),
        ('Max Time (ms)', 'max_time_ms', None),
        ('Std Dev (ms)', 'std_time_ms', None),
    ]

    for label, key, calc in metrics:
        pt_val = pytorch_results[key]
        trt_val = tensorrt_results[key]

        if calc == 'speedup':
            improvement = f"{pt_val/trt_val:.2f}x"
        else:
            improvement = ""

        print(f"{label:<30} {pt_val:>19.2f} {trt_val:>19.2f} {improvement:>15}")

    print()

    # FPS
    pt_fps = pytorch_results['fps']
    trt_fps = tensorrt_results['fps']
    fps_improvement = (trt_fps - pt_fps) / pt_fps * 100
    print(f"{'FPS':<30} {pt_fps:>19.1f} {trt_fps:>19.1f} {fps_improvement:>13.1f}%")

    print()

    # Memory
    print(f"{'GPU Memory (MB)':<30} {pytorch_results['gpu_memory_mb']:>19.2f} {tensorrt_results['gpu_memory_mb']:>19.2f}")

    print()

    # Detections
    print(f"{'Detections Found':<30} {pytorch_results['num_detections']:>19} {tensorrt_results['num_detections']:>19}")

    speedup = pytorch_results['avg_time_ms'] / tensorrt_results['avg_time_ms']

    print(f"\n{'='*90}")
    print(f"🚀 TensorRT is {speedup:.2f}x faster than PyTorch!")
    print(f"📊 FPS: {trt_fps:.1f} vs {pt_fps:.1f} (+{fps_improvement:.1f}%)")
    print(f"{'='*90}\n")


def save_results(
    pytorch_results: Dict,
    tensorrt_results: Dict,
    output_path: Path,
    format: str = 'txt'
) -> None:
    """Save benchmark results to file"""

    if format == 'json':
        # Save as JSON
        json_path = output_path.with_suffix('.json')
        data = {
            'pytorch': pytorch_results,
            'tensorrt': tensorrt_results,
            'speedup': pytorch_results['avg_time_ms'] / tensorrt_results['avg_time_ms']
        }

        with open(json_path, 'w') as f:
            json.dump(data, f, indent=2)

        logger.info(f"📄 Results saved to: {json_path}")

    else:
        # Save as text
        txt_path = output_path.with_suffix('.txt')
        speedup = pytorch_results['avg_time_ms'] / tensorrt_results['avg_time_ms']
        fps_improvement = (tensorrt_results['fps'] - pytorch_results['fps']) / pytorch_results['fps'] * 100

        with open(txt_path, 'w') as f:
            f.write("="*90 + "\n")
            f.write(" " * 25 + "TENSORRT BENCHMARK RESULTS\n")
            f.write("="*90 + "\n\n")

            f.write(f"PyTorch Model:  {pytorch_results['model_path']}\n")
            f.write(f"TensorRT Model: {tensorrt_results['model_path']}\n")
            f.write(f"Benchmark Runs: {pytorch_results['num_runs']}\n")
            f.write(f"Warmup Runs:    {pytorch_results['warmup_runs']}\n\n")

            f.write(f"{'Metric':<30} {'PyTorch':<20} {'TensorRT':<20} {'Improvement':>15}\n")
            f.write(f"{'-'*30} {'-'*20} {'-'*20} {'-'*15}\n")

            metrics = [
                ('Avg Inference Time (ms)', 'avg_time_ms', f"{speedup:.2f}x"),
                ('Min Time (ms)', 'min_time_ms', ''),
                ('Median Time (ms)', 'median_time_ms', ''),
                ('P95 Time (ms)', 'p95_time_ms', ''),
                ('P99 Time (ms)', 'p99_time_ms', ''),
                ('Max Time (ms)', 'max_time_ms', ''),
                ('Std Dev (ms)', 'std_time_ms', ''),
            ]

            for label, key, improvement in metrics:
                pt_val = pytorch_results[key]
                trt_val = tensorrt_results[key]
                f.write(f"{label:<30} {pt_val:>19.2f} {trt_val:>19.2f} {improvement:>15}\n")

            f.write(f"\n{'FPS':<30} {pytorch_results['fps']:>19.1f} {tensorrt_results['fps']:>19.1f} {fps_improvement:>13.1f}%\n")
            f.write(f"{'GPU Memory (MB)':<30} {pytorch_results['gpu_memory_mb']:>19.2f} {tensorrt_results['gpu_memory_mb']:>19.2f}\n")
            f.write(f"{'Detections Found':<30} {pytorch_results['num_detections']:>19} {tensorrt_results['num_detections']:>19}\n\n")

            f.write("="*90 + "\n")
            f.write(f"TensorRT is {speedup:.2f}x faster than PyTorch!\n")
            f.write(f"Achieves {tensorrt_results['fps']:.1f} FPS vs {pytorch_results['fps']:.1f} FPS (+{fps_improvement:.1f}%)\n")
            f.write("="*90 + "\n")

        logger.info(f"📄 Results saved to: {txt_path}")


def main():
    """Main benchmark function with CLI"""

    parser = argparse.ArgumentParser(
        description='Benchmark PyTorch vs TensorRT YOLO models',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        '--pytorch-model',
        type=str,
        default='/home/jay/Project/runs/train/yolo11n_pepper_gpu_100epochs/weights/best.pt',
        help='Path to PyTorch model (.pt file)'
    )

    parser.add_argument(
        '--tensorrt-model',
        type=str,
        default='/home/jay/Project/runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine',
        help='Path to TensorRT model (.engine file)'
    )

    parser.add_argument(
        '--image',
        type=str,
        help='Path to test image (if not provided, uses first image from val set)'
    )

    parser.add_argument(
        '--runs',
        type=int,
        default=100,
        help='Number of benchmark runs'
    )

    parser.add_argument(
        '--warmup',
        type=int,
        default=10,
        help='Number of warmup runs'
    )

    parser.add_argument(
        '--output',
        type=str,
        default='tensorrt_benchmark_results',
        help='Output file path (without extension)'
    )

    parser.add_argument(
        '--format',
        choices=['txt', 'json', 'both'],
        default='both',
        help='Output format'
    )

    args = parser.parse_args()

    # Print header
    print(f"""
╔══════════════════════════════════════════════════════════════════╗
║          PyTorch vs TensorRT Performance Benchmark              ║
╚══════════════════════════════════════════════════════════════════╝

Configuration:
  - PyTorch Model:  {Path(args.pytorch_model).name}
  - TensorRT Model: {Path(args.tensorrt_model).name}
  - Benchmark Runs: {args.runs} (after {args.warmup} warmup runs)
""")

    # Verify PyTorch model exists
    pytorch_model = Path(args.pytorch_model)
    if not pytorch_model.exists():
        logger.error(f"❌ PyTorch model not found: {pytorch_model}")
        sys.exit(1)

    # Verify TensorRT model exists
    tensorrt_model = Path(args.tensorrt_model)
    if not tensorrt_model.exists():
        logger.error(f"❌ TensorRT model not found: {tensorrt_model}")
        sys.exit(1)

    # Find test image
    if args.image:
        test_image = Path(args.image)
        if not test_image.exists():
            logger.error(f"❌ Test image not found: {test_image}")
            sys.exit(1)
    else:
        # Use first image from val set
        val_dir = Path("/home/jay/Project/pepper_dataset/images/val")
        images = list(val_dir.glob("*.jpg"))
        if not images:
            logger.error(f"❌ No images found in {val_dir}")
            sys.exit(1)
        test_image = images[0]

    logger.info(f"  - Test Image:     {test_image.name}")

    # Run benchmarks
    logger.info("\nStarting benchmarks...")

    pytorch_results = benchmark_model(
        str(pytorch_model),
        str(test_image),
        num_runs=args.runs,
        warmup_runs=args.warmup
    )

    if pytorch_results is None:
        logger.error("❌ PyTorch benchmark failed")
        sys.exit(1)

    tensorrt_results = benchmark_model(
        str(tensorrt_model),
        str(test_image),
        num_runs=args.runs,
        warmup_runs=args.warmup
    )

    if tensorrt_results is None:
        logger.error("❌ TensorRT benchmark failed")
        sys.exit(1)

    # Print comparison
    print_comparison(pytorch_results, tensorrt_results)

    # Save results
    output_path = Path(args.output)

    if args.format in ['txt', 'both']:
        save_results(pytorch_results, tensorrt_results, output_path, format='txt')

    if args.format in ['json', 'both']:
        save_results(pytorch_results, tensorrt_results, output_path, format='json')

    logger.info("\n✅ Benchmark complete!")


if __name__ == "__main__":
    main()
