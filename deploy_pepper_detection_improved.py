#!/usr/bin/env python3
"""
Improved Pepper Detection Deployment Script (TensorRT)
Production-ready pepper detection system with enhanced features

Improvements:
- Auto-load class names from data.yaml
- Input validation
- Logging support
- CSV export for batch processing
- Better error handling
- Video codec selection
- Memory-efficient batch processing

Usage:
    # Detect single image
    python3 deploy_pepper_detection_improved.py --image path/to/image.jpg

    # Real-time camera detection
    python3 deploy_pepper_detection_improved.py --camera 0

    # Process video file
    python3 deploy_pepper_detection_improved.py --video path/to/video.mp4

    # Batch process folder with CSV export
    python3 deploy_pepper_detection_improved.py --folder path/to/images/ --csv results.csv

Author: Generated with Claude Code
Date: 2025-11-04
"""

import argparse
import csv
import logging
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Union

import cv2
import numpy as np
import yaml
from ultralytics import YOLO


# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)


def load_class_names(data_yaml_path: str) -> Optional[Dict[int, str]]:
    """
    Load class names from data.yaml file

    Args:
        data_yaml_path: Path to data.yaml file

    Returns:
        dict: Mapping of class ID to class name, or None if error
    """
    try:
        with open(data_yaml_path, 'r') as f:
            data = yaml.safe_load(f)

        if 'names' in data:
            return data['names']
        else:
            logger.warning(f"⚠️  'names' not found in {data_yaml_path}")
            return None

    except Exception as e:
        logger.error(f"❌ Failed to load class names: {e}")
        return None


class PepperDetector:
    """Enhanced TensorRT-based pepper detection system"""

    def __init__(
        self,
        model_path: str,
        conf_threshold: float = 0.5,
        device: int = 0,
        data_yaml: Optional[str] = None
    ):
        """
        Initialize pepper detector

        Args:
            model_path: Path to TensorRT engine file
            conf_threshold: Confidence threshold for detections (0-1)
            device: CUDA device ID
            data_yaml: Path to data.yaml file for class names
        """
        # Validate confidence threshold
        if not 0.0 <= conf_threshold <= 1.0:
            raise ValueError(f"conf_threshold must be between 0 and 1, got {conf_threshold}")

        logger.info(f"Loading TensorRT model: {model_path}")

        try:
            self.model = YOLO(model_path)
            logger.info("✅ Model loaded successfully!")
        except Exception as e:
            logger.error(f"❌ Failed to load model: {e}")
            raise

        self.conf_threshold = conf_threshold
        self.device = device

        # Load class names from data.yaml if provided
        if data_yaml and Path(data_yaml).exists():
            self.class_names = load_class_names(data_yaml)
            if self.class_names:
                logger.info(f"✅ Loaded {len(self.class_names)} class names from {data_yaml}")
        else:
            # Fallback to default class names
            self.class_names = {
                0: 'pepper_red_large',
                1: 'pepper_red_small',
                2: 'pepper_red_deformed',
                3: 'pepper_red_wrinkled',
                4: 'pepper_red_rotten',
                5: 'pepper_red_insect',
                6: 'pepper_green_medium',
                7: 'pepper_green_small',
                8: 'pepper_green_rotten',
                9: 'pepper_green_insect'
            }
            logger.warning("⚠️  Using default class names")

        # Color mapping for visualization
        self.class_colors = self._generate_colors(len(self.class_names))

        logger.info(f"   Confidence threshold: {conf_threshold}")
        logger.info(f"   Device: CUDA:{device}")

    def _generate_colors(self, num_classes: int) -> Dict[int, tuple]:
        """Generate distinct colors for each class"""
        colors = {}
        for i in range(num_classes):
            # Generate colors in HSV space for better distinction
            hue = int(180 * i / num_classes)
            color_hsv = np.uint8([[[hue, 255, 255]]])
            color_bgr = cv2.cvtColor(color_hsv, cv2.COLOR_HSV2BGR)[0][0]
            colors[i] = tuple(map(int, color_bgr))
        return colors

    def detect_image(
        self,
        image_path: str,
        save_path: Optional[str] = None,
        show: bool = True
    ) -> Dict:
        """
        Detect peppers in a single image

        Args:
            image_path: Path to input image
            save_path: Path to save annotated image (optional)
            show: Display result (default: True)

        Returns:
            dict: Detection results
        """
        logger.info(f"📷 Processing: {image_path}")

        if not Path(image_path).exists():
            logger.error(f"❌ Image not found: {image_path}")
            return {'error': 'Image not found'}

        try:
            start_time = time.perf_counter()

            # Run detection
            results = self.model(
                image_path,
                device=self.device,
                conf=self.conf_threshold,
                verbose=False
            )

            inference_time = (time.perf_counter() - start_time) * 1000  # ms

            # Get results
            result = results[0]
            detections = []

            for box in result.boxes:
                class_id = int(box.cls)
                conf = float(box.conf)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                detections.append({
                    'class_id': class_id,
                    'class_name': self.class_names.get(class_id, f'Class {class_id}'),
                    'confidence': conf,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)]
                })

            # Print detections
            logger.info(f"⏱️  Inference time: {inference_time:.1f} ms")
            logger.info(f"🎯 Detections: {len(detections)}")

            for i, det in enumerate(detections, 1):
                logger.info(f"   {i}. {det['class_name']}: {det['confidence']:.2%}")

            # Save annotated image
            if save_path or show:
                annotated = result.plot()

                if save_path:
                    cv2.imwrite(str(save_path), annotated)
                    logger.info(f"💾 Saved to: {save_path}")

                if show and not sys.platform.startswith('linux'):
                    # Only show on platforms with display
                    # Resize for display if too large
                    h, w = annotated.shape[:2]
                    if w > 1280 or h > 720:
                        scale = min(1280/w, 720/h)
                        new_w = int(w * scale)
                        new_h = int(h * scale)
                        annotated = cv2.resize(annotated, (new_w, new_h))

                    cv2.imshow('Pepper Detection', annotated)
                    logger.info("Press any key to close...")
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

            return {
                'detections': detections,
                'inference_time_ms': inference_time,
                'num_detections': len(detections),
                'image_path': str(image_path)
            }

        except Exception as e:
            logger.error(f"❌ Detection failed: {e}")
            return {'error': str(e)}

    def detect_video(
        self,
        video_source: Union[int, str],
        save_path: Optional[str] = None,
        display: bool = True,
        skip_frames: int = 0
    ) -> Optional[Dict]:
        """
        Real-time detection on video/camera

        Args:
            video_source: Camera ID (int) or video file path (str)
            save_path: Path to save output video (optional)
            display: Show real-time display (default: True)
            skip_frames: Skip N frames between detections (0 = process all)

        Returns:
            dict: Video processing statistics
        """
        logger.info(f"🎥 Opening video source: {video_source}")

        try:
            cap = cv2.VideoCapture(video_source)

            if not cap.isOpened():
                logger.error(f"❌ Error: Could not open video source")
                return None

            # Get video properties
            fps = cap.get(cv2.CAP_PROP_FPS)
            if fps == 0:
                fps = 30  # Default FPS for cameras
                logger.warning(f"⚠️  Could not read FPS, using default: {fps}")

            width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

            logger.info(f"   Resolution: {width}x{height}")
            logger.info(f"   FPS: {fps:.1f}")
            if total_frames > 0:
                logger.info(f"   Total frames: {total_frames}")

            # Video writer
            writer = None
            if save_path:
                # Try multiple codecs
                for codec in ['avc1', 'mp4v', 'XVID']:
                    fourcc = cv2.VideoWriter_fourcc(*codec)
                    writer = cv2.VideoWriter(str(save_path), fourcc, fps, (width, height))
                    if writer.isOpened():
                        logger.info(f"💾 Saving to: {save_path} (codec: {codec})")
                        break
                    else:
                        writer.release()
                        writer = None

                if writer is None:
                    logger.warning("⚠️  Could not create video writer, saving disabled")

            frame_count = 0
            processed_count = 0
            start_time = time.perf_counter()
            inference_times = []

            logger.info("\n▶️  Processing video... (Press 'q' to quit)")

            while True:
                ret, frame = cap.read()
                if not ret:
                    break

                frame_count += 1

                # Skip frames if requested
                if skip_frames > 0 and (frame_count - 1) % (skip_frames + 1) != 0:
                    continue

                # Run detection
                inf_start = time.perf_counter()
                results = self.model(
                    frame,
                    device=self.device,
                    conf=self.conf_threshold,
                    verbose=False
                )
                inf_time = (time.perf_counter() - inf_start) * 1000

                inference_times.append(inf_time)
                processed_count += 1

                # Annotate frame
                annotated = results[0].plot()

                # Calculate and display stats
                elapsed = time.perf_counter() - start_time
                current_fps = processed_count / elapsed if elapsed > 0 else 0

                # Add FPS overlay
                cv2.putText(
                    annotated,
                    f"FPS: {current_fps:.1f} | Inference: {inf_time:.1f}ms",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # Add detection count
                num_detections = len(results[0].boxes)
                cv2.putText(
                    annotated,
                    f"Detections: {num_detections}",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2
                )

                # Display
                if display:
                    cv2.imshow('Pepper Detection', annotated)

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        logger.info("\n⏸️  Stopped by user")
                        break

                # Save
                if writer and writer.isOpened():
                    writer.write(annotated)

                # Progress update
                if processed_count % 30 == 0:
                    avg_inf = np.mean(inference_times) if inference_times else 0
                    logger.info(f"   Processed {processed_count} frames | {current_fps:.1f} FPS | Avg inference: {avg_inf:.1f}ms")

        except KeyboardInterrupt:
            logger.info("\n⏸️  Interrupted by user")

        except Exception as e:
            logger.error(f"❌ Video processing error: {e}")
            return None

        finally:
            cap.release()
            if writer and writer.isOpened():
                writer.release()
            cv2.destroyAllWindows()

        # Calculate statistics
        total_time = time.perf_counter() - start_time
        avg_fps = processed_count / total_time if total_time > 0 else 0
        avg_inference = np.mean(inference_times) if inference_times else 0

        logger.info(f"\n📊 Video Processing Complete!")
        logger.info(f"   Total frames: {frame_count}")
        logger.info(f"   Processed frames: {processed_count}")
        logger.info(f"   Total time: {total_time:.1f}s")
        logger.info(f"   Average FPS: {avg_fps:.1f}")
        logger.info(f"   Average inference: {avg_inference:.1f}ms")

        return {
            'frame_count': frame_count,
            'processed_count': processed_count,
            'total_time_s': total_time,
            'avg_fps': avg_fps,
            'avg_inference_ms': avg_inference
        }

    def detect_folder(
        self,
        folder_path: str,
        output_folder: Optional[str] = None,
        save_results: bool = True,
        csv_path: Optional[str] = None
    ) -> Optional[Dict]:
        """
        Batch process all images in a folder

        Args:
            folder_path: Input folder containing images
            output_folder: Output folder for annotated images
            save_results: Save annotated images (default: True)
            csv_path: Path to save CSV results (optional)

        Returns:
            dict: Batch processing statistics
        """
        folder = Path(folder_path)
        logger.info(f"📁 Processing folder: {folder}")

        if not folder.exists():
            logger.error(f"❌ Folder not found: {folder}")
            return None

        # Find all images
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
        images = []
        for ext in image_extensions:
            images.extend(folder.glob(f'*{ext}'))
            images.extend(folder.glob(f'*{ext.upper()}'))

        logger.info(f"   Found {len(images)} images")

        if len(images) == 0:
            logger.error("❌ No images found!")
            return None

        # Create output folder
        if save_results:
            if output_folder is None:
                output_folder = folder / 'detected'
            output_folder = Path(output_folder)
            output_folder.mkdir(exist_ok=True, parents=True)
            logger.info(f"💾 Saving to: {output_folder}")

        # Prepare CSV if requested
        csv_file = None
        csv_writer = None
        if csv_path:
            csv_file = open(csv_path, 'w', newline='')
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow([
                'image_path',
                'num_detections',
                'inference_time_ms',
                'detections_detail'
            ])
            logger.info(f"📊 CSV output: {csv_path}")

        # Process images
        total_detections = 0
        inference_times = []
        start_time = time.perf_counter()

        logger.info("\n▶️  Processing images...")

        try:
            for i, image_path in enumerate(images, 1):
                # Detect
                inf_start = time.perf_counter()
                results = self.model(
                    str(image_path),
                    device=self.device,
                    conf=self.conf_threshold,
                    verbose=False
                )
                inf_time = (time.perf_counter() - inf_start) * 1000
                inference_times.append(inf_time)

                # Get detections
                num_dets = len(results[0].boxes)
                total_detections += num_dets

                # Collect detection details for CSV
                if csv_writer:
                    detection_details = []
                    for box in results[0].boxes:
                        class_id = int(box.cls)
                        conf = float(box.conf)
                        class_name = self.class_names.get(class_id, f'Class {class_id}')
                        detection_details.append(f"{class_name}:{conf:.2f}")

                    csv_writer.writerow([
                        str(image_path),
                        num_dets,
                        f"{inf_time:.2f}",
                        "; ".join(detection_details)
                    ])

                # Save
                if save_results:
                    annotated = results[0].plot()
                    save_path = output_folder / image_path.name
                    cv2.imwrite(str(save_path), annotated)

                # Progress
                if i % 10 == 0 or i == len(images):
                    avg_inf = np.mean(inference_times)
                    logger.info(f"   Progress: {i}/{len(images)} | Avg inference: {avg_inf:.1f}ms")

        except Exception as e:
            logger.error(f"❌ Batch processing error: {e}")
            return None

        finally:
            if csv_file:
                csv_file.close()

        total_time = time.perf_counter() - start_time
        avg_inference = np.mean(inference_times)

        logger.info(f"\n✅ Batch Processing Complete!")
        logger.info(f"   Images processed: {len(images)}")
        logger.info(f"   Total detections: {total_detections}")
        logger.info(f"   Total time: {total_time:.1f}s")
        logger.info(f"   Average inference: {avg_inference:.1f}ms")
        logger.info(f"   Throughput: {len(images)/total_time:.1f} images/second")

        return {
            'num_images': len(images),
            'total_detections': total_detections,
            'total_time_s': total_time,
            'avg_inference_ms': avg_inference,
            'throughput_ips': len(images) / total_time
        }


def main():
    """Main CLI interface"""

    parser = argparse.ArgumentParser(
        description='Improved Pepper Detection using TensorRT',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Detect single image
  python3 deploy_pepper_detection_improved.py --image test.jpg

  # Real-time camera (default camera)
  python3 deploy_pepper_detection_improved.py --camera 0

  # Process video file
  python3 deploy_pepper_detection_improved.py --video input.mp4 --save output.mp4

  # Batch process folder with CSV export
  python3 deploy_pepper_detection_improved.py --folder images/ --output results/ --csv detections.csv
        """
    )

    # Input options (mutually exclusive)
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--image', type=str, help='Single image file')
    input_group.add_argument('--camera', type=int, help='Camera device ID (0, 1, ...)')
    input_group.add_argument('--video', type=str, help='Video file path')
    input_group.add_argument('--folder', type=str, help='Folder containing images')

    # Model options
    parser.add_argument(
        '--model',
        type=str,
        default='runs/train/yolo11n_pepper_gpu_100epochs/weights/best.engine',
        help='Path to TensorRT model (.engine file)'
    )

    parser.add_argument(
        '--data-yaml',
        type=str,
        default='pepper_dataset/data.yaml',
        help='Path to data.yaml for class names'
    )

    parser.add_argument(
        '--conf',
        type=float,
        default=0.5,
        help='Confidence threshold (0-1, default: 0.5)'
    )

    parser.add_argument(
        '--device',
        type=int,
        default=0,
        help='CUDA device ID (default: 0)'
    )

    # Output options
    parser.add_argument('--save', type=str, help='Save output to file')
    parser.add_argument('--output', type=str, help='Output folder (for batch processing)')
    parser.add_argument('--csv', type=str, help='CSV file for batch results')
    parser.add_argument('--no-display', action='store_true', help='Do not display results')
    parser.add_argument('--skip-frames', type=int, default=0, help='Skip N frames in video processing (0=process all)')

    args = parser.parse_args()

    # Print header
    print("╔═══════════════════════════════════════════════════════════╗")
    print("║   Improved Pepper Detection System (TensorRT Optimized)  ║")
    print("╚═══════════════════════════════════════════════════════════╝")

    # Validate confidence threshold
    if not 0.0 <= args.conf <= 1.0:
        logger.error(f"❌ Confidence threshold must be between 0 and 1, got {args.conf}")
        sys.exit(1)

    # Check if model exists
    model_path = Path(args.model)
    if not model_path.exists():
        logger.error(f"❌ Model not found: {model_path}")
        sys.exit(1)

    # Initialize detector
    try:
        detector = PepperDetector(
            model_path=str(model_path),
            conf_threshold=args.conf,
            device=args.device,
            data_yaml=args.data_yaml
        )
    except Exception as e:
        logger.error(f"❌ Failed to initialize detector: {e}")
        sys.exit(1)

    # Process based on input type
    try:
        if args.image:
            # Single image
            detector.detect_image(
                image_path=args.image,
                save_path=args.save,
                show=not args.no_display
            )

        elif args.camera is not None:
            # Real-time camera
            detector.detect_video(
                video_source=args.camera,
                save_path=args.save,
                display=not args.no_display,
                skip_frames=args.skip_frames
            )

        elif args.video:
            # Video file
            detector.detect_video(
                video_source=args.video,
                save_path=args.save,
                display=not args.no_display,
                skip_frames=args.skip_frames
            )

        elif args.folder:
            # Batch processing
            detector.detect_folder(
                folder_path=args.folder,
                output_folder=args.output,
                save_results=True,
                csv_path=args.csv
            )

        logger.info("\n✅ Done!")

    except Exception as e:
        logger.error(f"❌ Processing failed: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
