[![DOI](https://zenodo.org/badge/247666595.svg)](https://zenodo.org/badge/latestdoi/247666595)

# Matlab Scripts
All the scripts that were used to obtained the results described in the paper are inside the
`matlab/matlab` directory.

To run them, simply go to the `matlab/matlab` with Matlab, and run the scripts
corresponding to the different sections.

The scripts have been developed and tested with Matlab 2020a.

# Paper
All the figures are inside the `inkscape` folder while the files used to compile
the paper are inside the `paper` folder.

To compile the paper, one can use the following commands:
``` bash
  cd paper;
  latexmk -pdflatex="pdflatex -synctex=1 -shell-escape -interaction nonstopmode" -pdf -bibtex -f paper.tex 
```

# Compile Figure

``` bash
  cd inkscape;
  for f in *.svg; do inkscape -D --export-type=pdf "$f"; done
```
