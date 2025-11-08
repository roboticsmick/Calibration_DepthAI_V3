#!/usr/bin/env python3
"""
Generate a ChArUco calibration board using modern OpenCV API.
Saves the board as a high-resolution image and PDF for printing.

Usage:
    python3 generate_charuco_board.py -nx 17 -ny 9 -s 4.0 -ms 3.0 -o charuco_board.pdf

Parameters:
    -nx: Number of squares in X direction (default: 17)
    -ny: Number of squares in Y direction (default: 9)
    -s: Square size in cm (default: 4.0)
    -ms: Marker size in cm (default: 3.0, which is 75% of square size)
    -o: Output filename (default: charuco_board_<nx>x<ny>.pdf)
"""

import argparse
import cv2
import numpy as np
from pathlib import Path


def generate_charuco_board(squares_x, squares_y, square_size_cm, marker_size_cm, output_file=None):
    """
    Generate a ChArUco board using modern OpenCV API and save it as PDF.

    Args:
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction
        square_size_cm: Size of each square in centimeters
        marker_size_cm: Size of each ArUco marker in centimeters
        output_file: Output filename (if None, auto-generated)

    Returns:
        Path to the generated file
    """

    # Validate marker size
    if marker_size_cm >= square_size_cm:
        raise ValueError(f"Marker size ({marker_size_cm}cm) must be smaller than square size ({square_size_cm}cm)")

    # Calculate marker border as percentage
    marker_border = (square_size_cm - marker_size_cm) / (2 * marker_size_cm)

    # Warn if border is too small (less than 70% of marker size)
    if marker_border < 0.35:  # 0.35 * 2 = 0.7 (70%)
        print(f"WARNING: Marker border ({marker_border*2*100:.1f}%) is less than 70% of marker size.")
        print(f"Consider increasing square size or decreasing marker size for better detection.")

    print(f"\nGenerating ChArUco board:")
    print(f"  Dimensions: {squares_x} x {squares_y} squares")
    print(f"  Square size: {square_size_cm} cm")
    print(f"  Marker size: {marker_size_cm} cm")
    print(f"  Marker border: {marker_border*marker_size_cm:.2f} cm ({marker_border*100:.1f}%)")

    # Create ArUco dictionary (same as used in calibration)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)

    # Create ChArUco board
    # Note: OpenCV uses tuple (x, y) for board size
    board = cv2.aruco.CharucoBoard(
        (squares_x, squares_y),
        square_size_cm,
        marker_size_cm,
        aruco_dict
    )

    # Calculate DPI for printing
    # We want high resolution: 300 DPI is standard for printing
    dpi = 300

    # Convert cm to inches (1 inch = 2.54 cm)
    board_width_inches = (squares_x * square_size_cm) / 2.54
    board_height_inches = (squares_y * square_size_cm) / 2.54

    # Calculate pixel dimensions
    img_width_px = int(board_width_inches * dpi)
    img_height_px = int(board_height_inches * dpi)

    print(f"\nBoard physical size:")
    print(f"  Width: {squares_x * square_size_cm:.1f} cm ({board_width_inches:.2f} inches)")
    print(f"  Height: {squares_y * square_size_cm:.1f} cm ({board_height_inches:.2f} inches)")
    print(f"\nImage resolution:")
    print(f"  {img_width_px} x {img_height_px} pixels at {dpi} DPI")

    # Generate the board image
    # We add a small margin (1 square) around the board for easier handling
    margin_size = int(square_size_cm / 2.54 * dpi)  # 0.5cm margin

    board_img = board.generateImage(
        (img_width_px, img_height_px),
        marginSize=margin_size,
        borderBits=1
    )

    # Generate output filename if not provided
    if output_file is None:
        output_file = f"charuco_board_{squares_x}x{squares_y}_{square_size_cm}cm.png"

    output_path = Path(output_file)

    # Save as PNG first (high quality)
    png_path = output_path.with_suffix('.png')
    cv2.imwrite(str(png_path), board_img)
    print(f"\n✓ Saved PNG image: {png_path}")

    # Try to save as PDF for easy printing
    try:
        # Convert to PDF using OpenCV or reportlab
        import matplotlib.pyplot as plt
        from matplotlib.backends.backend_pdf import PdfPages

        # Create PDF
        pdf_path = output_path.with_suffix('.pdf')

        with PdfPages(str(pdf_path)) as pdf:
            # Calculate figure size in inches
            fig_width = board_width_inches + 1  # Add 1 inch margin
            fig_height = board_height_inches + 1

            fig = plt.figure(figsize=(fig_width, fig_height))
            ax = fig.add_subplot(111)
            ax.imshow(board_img, cmap='gray')
            ax.axis('off')

            # Add text with board information
            info_text = (
                f"ChArUco Board: {squares_x}x{squares_y}\n"
                f"Square: {square_size_cm}cm | Marker: {marker_size_cm}cm\n"
                f"Print at 100% scale (no scaling!)\n"
                f"Measure actual square size after printing"
            )
            fig.text(0.5, 0.02, info_text, ha='center', fontsize=10,
                    family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

            plt.tight_layout()
            pdf.savefig(fig, dpi=dpi)
            plt.close()

        print(f"✓ Saved PDF: {pdf_path}")
        print(f"\n📋 PRINTING INSTRUCTIONS:")
        print(f"  1. Open the PDF file: {pdf_path}")
        print(f"  2. Print at 100% scale (DO NOT scale to fit page)")
        print(f"  3. Measure a square with a ruler - it should be EXACTLY {square_size_cm} cm")
        print(f"  4. If the size is wrong, adjust printer settings and print again")
        print(f"  5. Mount the printed board on a flat, rigid surface (cardboard, foam board, etc.)")
        print(f"  6. Use the MEASURED square size in your calibration command")

        return pdf_path

    except ImportError:
        print("\nNote: matplotlib not found. PDF generation skipped.")
        print("Install matplotlib for PDF support: pip install matplotlib")
        print(f"\nYou can still use the PNG file: {png_path}")
        print("Print it using an image viewer, ensuring 100% scale / no scaling")

        return png_path


def main():
    parser = argparse.ArgumentParser(
        description="Generate ChArUco calibration board for camera calibration",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Generate a 17x9 board with 4cm squares and 3cm markers
  python3 generate_charuco_board.py -nx 17 -ny 9 -s 4.0 -ms 3.0

  # Generate a 11x8 board (default) with auto-sized markers
  python3 generate_charuco_board.py -s 3.0

  # Custom output filename
  python3 generate_charuco_board.py -nx 17 -ny 9 -s 4.0 -o my_board.pdf

Note: Marker size defaults to 75% of square size if not specified.
        """
    )

    parser.add_argument(
        '-nx', '--squares-x',
        type=int,
        default=11,
        help='Number of squares in X direction (default: 11)'
    )

    parser.add_argument(
        '-ny', '--squares-y',
        type=int,
        default=8,
        help='Number of squares in Y direction (default: 8)'
    )

    parser.add_argument(
        '-s', '--square-size',
        type=float,
        required=True,
        help='Square size in centimeters (e.g., 4.0)'
    )

    parser.add_argument(
        '-ms', '--marker-size',
        type=float,
        default=None,
        help='Marker size in centimeters (default: 75%% of square size)'
    )

    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help='Output filename (default: auto-generated based on board size)'
    )

    args = parser.parse_args()

    # Calculate marker size if not provided (75% of square size)
    marker_size = args.marker_size if args.marker_size is not None else args.square_size * 0.75

    # Validate inputs
    if args.squares_x < 4 or args.squares_y < 4:
        parser.error("Board must have at least 4x4 squares")

    if args.square_size < 1.0:
        parser.error("Square size must be at least 1.0 cm")

    if marker_size >= args.square_size:
        parser.error(f"Marker size ({marker_size}cm) must be smaller than square size ({args.square_size}cm)")

    # Generate the board
    try:
        output_file = generate_charuco_board(
            args.squares_x,
            args.squares_y,
            args.square_size,
            marker_size,
            args.output
        )

        print(f"\n✓ SUCCESS! Board generated successfully.")
        print(f"\nTo use this board for calibration:")
        print(f"  python3 ssh_camera_filter_control_calibrate_V3.py \\")
        print(f"    -s {args.square_size} \\")
        print(f"    -ms {marker_size} \\")
        print(f"    -nx {args.squares_x} \\")
        print(f"    -ny {args.squares_y} \\")
        print(f"    -brd OAK-FFC-3P-HQ113.json \\")
        print(f"    --ssh-preview")

    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
