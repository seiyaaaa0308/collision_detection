# Collision Detection

## DEMO

![git_ref](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/96dfbf87-6b5f-4468-a765-50313ae1bf47)


## Features

Using this code you can execute collision detection between Convex-Hull and Point Cloud.
In this code, Medical Robot(Convex-Hull) vs Patient(Point Cloud) is simulated.

- There are second phases in detection method.
1. Using AABB (Points outside AABB are removed)
2. Using Convex-Hull (Points outside Convex-Hull are removed)

After completing processing, points that exist are inside of Convex-Hull and judged as collision point.

## Requirement

* open3d 0.17.0
* numpy 1.26.0


## Author

* Seiya Kobayashi
* Okayama univ student
* pkjf2jc9@s.okayama-u.ac.jp
