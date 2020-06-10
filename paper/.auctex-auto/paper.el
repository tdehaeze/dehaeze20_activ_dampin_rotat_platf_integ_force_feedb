(TeX-add-style-hook
 "paper"
 (lambda ()
   (TeX-add-to-alist 'LaTeX-provided-package-options
                     '(("inputenc" "utf8") ("fontenc" "T1") ("ulem" "normalem") ("tcolorbox" "most") ("babel" "USenglish" "english")))
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "path")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "url")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "nolinkurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperbaseurl")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperimage")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "hyperref")
   (add-to-list 'LaTeX-verbatim-macros-with-braces-local "href")
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
    "sec:org335669b"
    "sec:introduction"
    "sec:org8b756e7"
    "sec:theory"
    "sec:orgbf4a596"
    "fig:rotating_xy_platform"
    "sec:orgaa8880a"
    "eq:energy_inertial_frame"
    "eq:lagrangian_inertial_frame"
    "sec:org754b644"
    "sec:org9cbf82a"
    "sec:org8d24de3"
    "sec:conclusion"
    "sec:orgb252937")
   (LaTeX-add-bibliographies
    "ref"))
 :latex)

