[![DOI](https://zenodo.org/badge/247666595.svg)](https://zenodo.org/badge/latestdoi/247666595)

# Compile Figure

``` bash
  cd inkscape;
  for f in *.svg; do inkscape -D --export-type=pdf "$f"; done
```
