# LAB5 result tables

## D5/D6.a box.png -> box_in_scene.png

| Statistics | SIFT | SURF | ORB | FAST+BRIEF |
|---|---|---|---|---|
| # of Keypoints in Img 1 | 604 | 997 | 3141 | 2266 |
| # of Keypoints in Img 2 | 969 | 1822 | 4362 | 3653 |
| # of Matches | 604 | 997 | 3141 | 2266 |
| # of Good Matches | 102 | 133 | 120 | 41 |
| # of Inliers | 82 | 85 | 80 | 17 |
| Inlier Ratio | 80.4% | 63.9% | 66.7% | 41.5% |
| Time per pair (ms) | 45.5 | 59.0 | 25.0 | 15.0 |
| # of samples | 1 | 1 | 1 | 1 |

## D6.a parameters tried on the pair of images

| Variant | Kp 1 | Kp 2 | Good | Inliers | Ratio | Time (ms) |
|---|---|---|---|---|---|---|
| SIFT contrast 0.04 (default) | 604 | 969 | 102 | 82 | 80.4% | 45.5 |
| SIFT contrast 0.02 | 657 | 1181 | 102 | 82 | 80.4% | 46.3 |
| SURF hessian 100 (default) | 997 | 1822 | 133 | 85 | 63.9% | 59.0 |
| SURF hessian 50 | 1051 | 2133 | 149 | 89 | 59.7% | 64.1 |
| SURF hessian 400 | 786 | 1040 | 116 | 74 | 63.8% | 47.4 |
| ORB 500 features, 8 levels | 453 | 500 | 30 | 28 | 93.3% | 8.1 |
| ORB 500 features, 1 level (skeleton) | 500 | 500 | 7 | 7 | 100.0% | 2.8 |
| ORB 500 features, FLANN-LSH | 453 | 500 | 30 | 28 | 93.3% | 11.5 |
| ORB 2000 features | 1589 | 1999 | 74 | 56 | 75.7% | 12.0 |
| ORB 5000 features (used) | 3141 | 4362 | 120 | 80 | 66.7% | 25.0 |
| FAST threshold 10 (default) | 2266 | 3653 | 41 | 17 | 41.5% | 15.0 |
| FAST threshold 5 | 2698 | 5085 | 30 | 13 | 43.3% | 19.6 |
| FAST threshold 30 | 1095 | 1748 | 12 | 7 | 58.3% | 4.7 |
| FAST threshold 10, FLANN-LSH | 2266 | 3653 | 41 | 17 | 41.5% | 56.5 |

## D6.b/D7 30fps_424x240_2018-10-01-18-35-06.bag (average over frame pairs)

| Statistics | SIFT | SURF | ORB | FAST+BRIEF | Harris+LK |
|---|---|---|---|---|---|
| # of Keypoints in Img 1 | 403.6 | 467.0 | 952.0 | 826.8 | 374.4 |
| # of Keypoints in Img 2 | 403.6 | 466.9 | 951.8 | 826.7 | 367.8 |
| # of Matches | 403.6 | 467.0 | 952.0 | 826.8 | 367.8 |
| # of Good Matches | 258.4 | 334.1 | 698.6 | 633.5 | - |
| # of Inliers | 249.0 | 319.8 | 686.6 | 606.9 | 365.2 |
| Inlier Ratio | 95.7% | 95.3% | 98.1% | 95.1% | 99.3% |
| Time per pair (ms) | 28.4 | 25.0 | 13.0 | 8.1 | 6.6 |
| # of samples | 1583 | 1583 | 1583 | 1583 | 1583 |

## D6.b/D7 vnav-lab5-smooth-trajectory.bag (average over frame pairs)

| Statistics | SIFT | SURF | ORB | FAST+BRIEF | Harris+LK |
|---|---|---|---|---|---|
| # of Keypoints in Img 1 | 147.9 | 1192.9 | 813.2 | 568.4 | 156.5 |
| # of Keypoints in Img 2 | 147.8 | 1191.8 | 812.2 | 567.8 | 152.5 |
| # of Matches | 147.9 | 1192.9 | 813.2 | 568.4 | 152.5 |
| # of Good Matches | 92.6 | 715.3 | 449.4 | 392.1 | - |
| # of Inliers | 80.5 | 610.7 | 404.6 | 356.7 | 143.5 |
| Inlier Ratio | 86.3% | 85.5% | 90.0% | 90.6% | 94.7% |
| Time per pair (ms) | 81.6 | 85.2 | 20.1 | 6.8 | 12.3 |
| # of samples | 1385 | 1385 | 1385 | 1385 | 1385 |
