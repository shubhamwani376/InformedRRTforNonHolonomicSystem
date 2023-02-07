## Informed RRT for Nonholonomic systems - Car Parking assist
### Authors:
[Shubham Wani](https://www.linkedin.com/in/shubhamwani/) , Linfeng Li and Yang Liu

## Implementation
This project implements the following features:
1.Motion primitive generation using bicycle model to grow tree.
2.Find closest point in RR Tree and extend trees to cover entire area. 
3.Structs used to store data for nodes i.e. distance, parents, cost etc.
4.Sampling area decreases and the shortest path among those found is shown.

## Conclusion
1. Tested in a simulated environment and was found to be successful at guiding vehicles into parking spots and finding the shortest path to a destination.
2. Ellipse-Informed RRT and goal-sampling optimizations were used to improve the efficiency and accuracy of the system. The results
demonstrate the potential of this technology to
improve the safety, efficiency, and
cost-effectiveness of parking structures and
autonomous vehicles.
4. Refer to the [REPORT](https://github.com/shubhamwani376/InformedRRTforNonHolonomicSystem/blob/main/Informed%20RRT%20Project%20Report.pdf) for detailed conclusions and plots


## Video
![RRT](https://github.com/shubhamwani376/InformedRRTforNonHolonomicSystem/blob/main/Images/RRTwithObstacle.gif)
![IRRT](https://github.com/shubhamwani376/InformedRRTforNonHolonomicSystem/blob/main/Images/RRTwithObstacle.gif)

## Dependencies
```
MATLAB, R2020a ed. Natick, Massachusetts: The MathWorks Inc.
```
## Reference
1. J. D. Gammell, S. S. Srinivasa and T. D.Barfoot, "Informed RRT" doi: 10.1109/IROS.2014.6942976.
2. Rapidly-Exploring Random Trees: A New Tool for Path Planning, Steven L Lavalle.
