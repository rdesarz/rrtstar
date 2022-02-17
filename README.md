# Python RRT*

A Python implementation of the RRT*, an optimal version of the Rapidly Exploring Random Tree. Implementation is based on the following paper: <div class="csl-entry">Karaman, S., &#38; Frazzoli, E. (2010). Optimal kinodynamic motion planning using incremental sampling-based methods. <i>Proceedings of the IEEE Conference on Decision and Control</i>, 7681–7687. https://doi.org/10.1109/CDC.2010.5717430</div>. 

RRT* is a motion planning algorithm that allows to find an optimal path from an initial state to a goal region in the state space. 

This project allows to study the effect of the different parameters on the obtained path as well as changing easily some part of the algorithm such as the steering function.

## Installation and how-to-use

Clone the repository

    git clone git@github.com:rdesarz/rrtstar.git
  
Install the project using pip. You can use the -e option to edit the parameters of the algorithm
    
    python3 -m pip install -e rrtstar
  
Parameters can be changed in the `commande_line.py` file

To run the planner, type the following command

    rrtstar

