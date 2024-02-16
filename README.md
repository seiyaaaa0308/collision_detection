# Collision Detection Application

## DEMO

![Setup screen](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/775e904b-a64d-4b2b-93ce-5abeac3a4e28)

![Dicom viewer](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/6507780a-ca59-4560-bf8c-e98678ce1d42)

![Pointing mode](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/c0a63799-07b8-4fa4-8584-830a3326a800)

![Collision detection](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/3b4c7d2c-8f66-4b0f-9183-c780dd8243f4)

## Features
This GUI is for simulating minimally invasive treatment using a medical robot. 
The GUI has a DICOM viewer function, pointing function, and contact detection function. 
We are currently developing this system with the goal of using it for safety checks before surgery.

In this GUI collision detection between Convex-Hull and Point Cloud is executed.
In this code, Medical Robot(Convex-Hull) vs Patient(Point Cloud) is simulated.

- There are second phases in detection method.
1. Using AABB (Points outside AABB are removed)
2. Using Convex-Hull (Points outside Convex-Hull are removed)

After completing processing, points that exist are inside of Convex-Hull and judged as collision point.

![Collision detection](https://github.com/seiyaaaa0308/collision_detection/assets/127831728/f02c3426-a35b-46ad-9a73-0c9d21edb344)


## Author

* Seiya Kobayashi
* Okayama univ student
* pkjf2jc9@s.okayama-u.ac.jp
