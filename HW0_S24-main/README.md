# HW0

[HW0 + Julia Video Walkthrough](https://www.youtube.com/watch?v=RetAn_9AOMg)

## Getting Started
All the homeworks are distributed as Jupyter notebooks. Follow these instructions to get everything set up.

1. Install Julia. Download v1.6.7 from the [Julia website](https://julialang.org/downloads/#long_term_support_release). From here you can follow these [platform specific instructions](https://julialang.org/downloads/platform/) to get started. 
2. Clone this repo and put it wherever you want.
2. Start a Julia REPL in your terminal using `julia`, or the location to the Julia binaries.  If this doesn't work, make sure you followed the [platform specific instructions](https://julialang.org/downloads/platform/).
3. Install the [IJulia](https://github.com/JuliaLang/IJulia.jl) using the Julia package manager. In the REPL, enter the package manager using `]`, then enter `add IJulia` to add it to your system path.
4. In the REPL (hit backspace to exit the package manager), enter `using IJulia`
5. Launch the notebook using `notebook()` or `jupyterlab()`

## HW Instructions 

Fill out `Q1.ipynb` and `Q2.ipynb`.

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
