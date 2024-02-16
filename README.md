# Collision Detection

## DEMO

![Setup screen](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/775e904b-a64d-4b2b-93ce-5abeac3a4e28)

![Dicom viewer](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/6507780a-ca59-4560-bf8c-e98678ce1d42)

![Pointing mode](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/c0a63799-07b8-4fa4-8584-830a3326a800)

![Collision detection](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/3b4c7d2c-8f66-4b0f-9183-c780dd8243f4)

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
