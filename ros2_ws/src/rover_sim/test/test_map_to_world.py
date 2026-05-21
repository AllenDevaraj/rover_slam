import xml.etree.ElementTree as ET
from rover_sim.map_to_world import (
    parse_pgm, occupied_grid, merge_row_runs, runs_to_boxes, build_sdf,
    greedy_rectangles, rects_to_boxes,
)


def test_greedy_rectangles_merges_block():
    # a 2x2 occupied block (cols 0-1, rows 0-1) -> one 2x2 rectangle
    grid = [[True, True, False],
            [True, True, False],
            [False, False, False]]
    rects = greedy_rectangles(grid)
    assert (0, 0, 2, 2) in rects
    assert len(rects) == 1


def test_rects_to_boxes_min_cells_filter():
    grid = [[True, False], [False, False]]   # single isolated cell
    rects = greedy_rectangles(grid)
    assert rects_to_boxes(rects, 0.05, (0.0, 0.0), 2, min_cells=3) == []
    assert len(rects_to_boxes(rects, 0.05, (0.0, 0.0), 2, min_cells=1)) == 1


def _tiny_pgm(tmp_path):
    # 4x3 P5 PGM: a 2-cell horizontal wall (value 0=occupied) on the top row,
    # everything else free (254). Row-major, origin bottom-left after y-flip.
    w, h = 4, 3
    pix = [254] * (w * h)
    pix[0] = 0   # (row0,col0)
    pix[1] = 0   # (row0,col1)
    data = f'P5\n{w} {h}\n255\n'.encode() + bytes(pix)
    p = tmp_path / 'm.pgm'
    p.write_bytes(data)
    return str(p), w, h


def test_parse_pgm(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    gw, gh, pix = parse_pgm(path)
    assert (gw, gh) == (w, h)
    assert len(pix) == w * h
    assert pix[0] == 0 and pix[2] == 254


def test_occupied_grid_thresholds(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    assert grid[0][0] is True and grid[0][1] is True
    assert grid[0][2] is False and grid[1][0] is False


def test_merge_row_runs(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    runs = merge_row_runs(grid)          # list of (row, col_start, length)
    assert (0, 0, 2) in runs
    assert len(runs) == 1


def test_runs_to_boxes_world_coords(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    runs = merge_row_runs(grid)
    boxes = runs_to_boxes(runs, resolution=0.5, origin=(0.0, 0.0), img_h=h)
    assert len(boxes) == 1
    b = boxes[0]
    # 2 cells * 0.5 m wide
    assert abs(b['sx'] - 1.0) < 1e-6
    assert abs(b['sy'] - 0.5) < 1e-6
    # row 0 is the TOP image row -> highest world y after flip
    assert abs(b['y'] - ((h - 0 - 0.5) * 0.5 + 0.0)) < 1e-6


def test_build_sdf_is_valid_xml_with_one_wall(tmp_path):
    path, w, h = _tiny_pgm(tmp_path)
    _, _, pix = parse_pgm(path)
    grid = occupied_grid(pix, w, h, negate=0, occupied_thresh=0.65)
    boxes = runs_to_boxes(merge_row_runs(grid), 0.5, (0.0, 0.0), h)
    sdf = build_sdf(boxes, wall_height=1.0, world_name='m')
    root = ET.fromstring(sdf)            # raises if malformed
    assert root.tag == 'sdf'
    assert len(root.findall('.//world')) == 1
    # one wall link inside the walls model
    assert len(root.findall(".//model[@name='walls']/link")) == 1
