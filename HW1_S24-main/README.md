# HW1_S23

**Due 2/8 11:59 PM Eastern.**

## Video walkthrough

YouTube link: [https://youtu.be/V7i0tQlBLlE](https://youtu.be/V7i0tQlBLlE)

[![HW1 Walkthrough](https://img.youtu.be/V7i0tQlBLlE.jpg)](https://youtu.be/V7i0tQlBLlE)


## Instructions
NOTE: You should not use any while loops in this homework. All of these concepts can and should be implemented with for loops for multiple reasons. Using while loops for things like Newton's method and linesearches is more difficult for both you and the TA's to debug.


Fill out project proposal form: [https://forms.gle/mqdpazLZ2ZnapAeCA](https://forms.gle/mqdpazLZ2ZnapAeCA)

In this homework we are going to explore topics in dynamics and optimization. Here is an overview of the problems:

1. Implement 3 explicit and 3 implicit integrators to simulate a double pendulum and examine the differences in accuracy. The goal of this problem is to start thinking about integrators as modular, and demonstrate the difference between explicit and implicit integrators. 

2. Write a well structured implementation of Newton's method with a backtracking linesearch. You will use this function to solve a simple constrained optimization problem, then you will use it to solve for the motor torques and configuration for a quadruped to balance on one leg. 

3. Implement a Quadratic Program (QP) solver using the Augmented Lagrangian method. This solver will then be used in a clever way to simulate a falling brick as it makes contact with the floor. 

4. Fill out project proposal form: [https://forms.gle/mqdpazLZ2ZnapAeCA](https://forms.gle/mqdpazLZ2ZnapAeCA)

## Submission Instructions 

Feel free to use any method you'd like to export your Jupyter notebook as a PDF (**with all the cell outputs shown**) and **submit on gradescope**. 

### HTML to PDF (Recommended)

We recommend this method of converting your Jupyter notebook to a PDF because it requires no additional installs (hopefully). It's slightly involved, but it is the most consistent in our experience.

1. Open the Jupyter notebook in your favorite **web browser** (not VS Code) with [IJulia](https://github.com/JuliaLang/IJulia.jl).
2. Make sure **all cell outputs are shown** including plots and unit tests' results.
3. In the top left corner of the Jupyter menu bar, do `File -> Save and Export Notebook As -> HTML`. It should download an HTML file.
4. Open the downloaded HTML file in your favorite web browser.
5. Open up the browser's print menu and select `Save as PDF`.
6. Save PDF and **submit on gradescope**.

### Others

If HTML to PDF does not work, feel free to try other methods: [https://mljar.com/blog/jupyter-notebook-pdf/](https://mljar.com/blog/jupyter-notebook-pdf/). 
