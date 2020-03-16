(TeX-add-style-hook
 "paper"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("inputenc" "utf8") ("fontenc" "T1") ("ulem" "normalem") ("tcolorbox" "most") ("babel" "USenglish" "english")))
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "href")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperref")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperimage")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperbaseurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "nolinkurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-delims-local "path")
   (TeX-run-style-hooks
    "latex2e"
    "config"
    "ISMA_USD2020"
    "ISMA_USD202010"
    "inputenc"
    "fontenc"
    "graphicx"
    "grffile"
    "longtable"
    "wrapfig"
    "rotating"
    "ulem"
    "amsmath"
    "textcomp"
    "amssymb"
    "capt-of"
    "hyperref"
    "tcolorbox"
    "bm"
    "booktabs"
    "tabularx"
    "array"
    "siunitx"
    "amsfonts"
    "cases"
    "algorithmic"
    "xcolor"
    "import"
    "babel")
   (LaTeX-add-labels
    "sec:org8c48899"
    "sec:introduction"
    "sec:org60f23e3"
    "sec:theory"
    "sec:orgd677659"
    "sec:conclusion"
    "sec:orgf333899")
   (LaTeX-add-bibliographies
    "ref"))
 :latex)

