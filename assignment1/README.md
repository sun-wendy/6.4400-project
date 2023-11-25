# Assignment 1: Curves & Surfaces
<br>

In this assignment, I implemented the following functionalities:
* Implemented curves in Bezier & B-Spline basis:
  * Wired the `B` key to convert control points geometry from Bezier to B-Spline basis;
  * Wired the `Z` key to convert geometry from B-Spline to Bezier basis;
  * Wired the `T` key to toggle spline basis;

* Implemented patches in bicubic Bezier & B-Spline tensor product patches;
* Added two extra artifacts.
<br><br>


### Compile & Run Code

I followed the steps on the assignment page to compile and run my code, and I didn't run into any compilation error.
<br><br>


### Collaboration

I went to Office Hours a couple times for debugging. I collaborated with Jiacheng Zhu, Karen Guo, and Liane Xu on some parts.
<br><br>


### References

I used references for creating three new artifacts **(the images could be found in [the Images folder](/images/))**.
* My first curve artifact is a circle. I referenced [this Stack Overflow page](https://stackoverflow.com/questions/1734745/how-to-create-circle-with-b%C3%A9zier-curves) as well as [this article](https://spencermortensen.com/articles/bezier-circle/) for calculating control points for the four curves that make up a circle.
* My second patch artifact is an abstract teacup (actually very abstract). My collaborator Jiacheng Zhu introduced me to [this web page](https://people.sc.fsu.edu/~jburkardt/data/bezier_surface/bezier_surface.html) that has a file for a teacup object.
* My third patch artifact is a bowl-shaped object. I built upon the pattern I found in the circle and the teapot splines to render the bowl.
<br><br>


### Known Problems with My Code

There is no known problem with my code.
<br><br>


### Extra Credit

I did one easy extra-credit problem on changing the material. I changed the material color to be based on the normal vector at the midpoint of each patch, so that the color of each patch depends on how slanted the patch's midpoint normal vector is. To change colors of all patches at the same time, I created a parent class <code>Surface</code> that adds all patch nodes as its children, and added the surface node to root.
<br><br>


### Comments

I enjoyed this assignment overall, as it really led me to understand the math underlying Bezier & B-Spline curves as well as patches. But the instructions for the last problem — creating two artifacts — were very vague, and I wasn't sure how to get started on computing control points for patches even though we could reference outside resources. It'd be better if it's an extra-credit problem.
