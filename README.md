# robot_emblem
A ROS 2 demo package inspired by *Fire Emblem*-style grid maps.   This project builds a simple **grid-based map** with three terrain types (`free`, `tree`, `ice`),   renders it in RViz using `MarkerArray`, and adds grid lines for clarity.   It is the foundation for later adding units, movement, and RPG logic.
### ----Day 1 ------

<img width="492" height="387" alt="53ee6f333d49435a19bb17c02e3c57d4" src="https://github.com/user-attachments/assets/d7467646-6a7a-4e08-a550-8a7be9c647b0" />

### -----Day 2 + 3------
<img width="522" height="320" alt="image" src="https://github.com/user-attachments/assets/d670b98f-ba2a-47e5-a2b8-6bd9559d22cc" />

### Day 2
- Implemented a simple movement logic.  
- Used RViz’s **Publish Point** tool to publish clicked points.  
- Subscribed to these points to update relative position.  
- The robot now moves one grid closer to the target each step, simulating basic movement.

### Day 3
- Replaced the placeholder marker with a simple **URDF-based robot model**.  
- Integrated the model into RViz through `robot_state_publisher`.  
- The robot model now follows the same grid-based movement logic, providing a more intuitive visualization.

### Day 4
<img width="452" height="300" alt="539a52dc3822fe17672034ab22cbf660" src="https://github.com/user-attachments/assets/05334e5c-1b0e-4d48-9713-e6264e06103f" />
<img width="474" height="313" alt="91192133922f29cb7779e9941f975201" src="https://github.com/user-attachments/assets/cc362174-4410-4fa9-83aa-7d0318936248" />

- Prevent diagonal movement (grid-based Manhattan steps only) 
- Add visualization of reachable area using BFS and marker array
- Next: refine movement logic, add turn-based system, and support turn state rollback"
