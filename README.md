# robot_emblem
A ROS 2 demo package inspired by *Fire Emblem*-style grid maps.   This project builds a simple **grid-based map** with three terrain types (`free`, `tree`, `ice`),   renders it in RViz using `MarkerArray`, and adds grid lines for clarity.   It is the foundation for later adding units, movement, and RPG logic.
### Day 1 

<img width="492" height="387" alt="53ee6f333d49435a19bb17c02e3c57d4" src="https://github.com/user-attachments/assets/d7467646-6a7a-4e08-a550-8a7be9c647b0" />

### Day 2

<img width="268" height="334" alt="image" src="https://github.com/user-attachments/assets/a305df75-79b4-479b-a3cd-f6f5729fbd9b" />

- Implemented a simple movement logic.  
- Used RViz’s **Publish Point** tool to publish clicked points.  
- Subscribed to these points to update relative position.  
- The robot now moves one grid closer to the target each step, simulating basic movement.

### Day 3
<img width="522" height="320" alt="image" src="https://github.com/user-attachments/assets/d670b98f-ba2a-47e5-a2b8-6bd9559d22cc" />

- Replaced the placeholder marker with a simple **URDF-based robot model**.  
- Integrated the model into RViz through `robot_state_publisher`.  
- The robot model now follows the same grid-based movement logic, providing a more intuitive visualization.

### Day 4
<img width="452" height="300" alt="539a52dc3822fe17672034ab22cbf660" src="https://github.com/user-attachments/assets/05334e5c-1b0e-4d48-9713-e6264e06103f" />
<img width="452" height="300" alt="91192133922f29cb7779e9941f975201" src="https://github.com/user-attachments/assets/cc362174-4410-4fa9-83aa-7d0318936248" />

- Prevent diagonal movement (grid-based Manhattan steps only) 
- Add visualization of reachable area using BFS and marker array
- Next: refine movement logic, add turn-based system, and support turn state rollback"

### Day 5-6

<img width="452" height="300" alt="9fd4d7423785197e41277f7fd94d5c81" src="https://github.com/user-attachments/assets/ddaae9ac-dac7-4041-a2dc-f68186163252" />
<img width="452" height="300" alt="725d3e4a941ec475860077ebc203d69e" src="https://github.com/user-attachments/assets/a2648cfc-bb88-4c42-bef6-85c4ba3e6c1e" />

- Added a second model (enemy unit) and verified that both the ally and enemy models can be spawned in RViz with distinct prefixes (ally/ and enemy/).

- Extended the UnitManagerNode logic:

    - Units are now stored in a vector, allowing multiple robots to be managed at once.

    - Each unit has its own frame (ally/base_link, enemy/base_link) and marker color (yellow for ally, red for enemy).

- Began introducing the turn-based concept:

    - Prototype currently only allows the ally unit to move when a point is clicked.

    - The enemy unit is present in the world but does not yet act during its turn.

    - This sets the foundation for alternating Player and Enemy phases in future iterations.
