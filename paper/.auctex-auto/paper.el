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
    "sec:orgd20252d"
    "sec:introduction"
    "sec:orgacbe1ae"
    "sec:org07e4fc8"
    "fig:rotating_xy_platform"
    "fig:cedrat_xy25xs"
    "sec:orgac1a52a"
    "eq:energy_inertial_frame"
    "eq:lagrangian_inertial_frame"
    "eq:lagrange_second_kind"
    "eq:eom_mixed"
    "eq:oem_coupled"
    "eq:du_coupled"
    "eq:dv_coupled"
    "sec:org47aaeee"
    "eq:coupledplant"
    "eq:coupled_plant"
    "eq:coupled_plant_no_rot"
    "fig:campbell_diagram"
    "fig:plant_compare_rotating_speed"
    "sec:org78c2eab"
    "sec:org6a00238"
    "sec:org5480f1b"
    "sec:orgbb0952e"
    "fig:root_locus_pure_iff"
    "sec:orgdb25e2c"
    "sec:org2985d35"
    "fig:loop_gain_modified_iff"
    "fig:root_locus_modified_iff"
    "fig:root_locus_wi_modified_iff"
    "sec:orga4142a5"
    "fig:rotating_xy_platform_springs"
    "fig:plant_iff_kp"
    "fig:root_locus_iff_kps"
    "fig:root_locus_iff_kp_bis"
    "fig:root_locus_opt_gain_iff_kp"
    "fig:plant_iff_compare_rotating_speed"
    "sec:org6a1be4f"
    "fig:root_locus_dvf"
    "sec:orga9658c0"
    "fig:comp_root_locus"
    "fig:comp_compliance"
    "fig:comp_transmissibility"
    "sec:orgcdf948f"
    "sec:conclusion"
    "sec:org6c21e13")
   (LaTeX-add-environments
    '("IEEEbiography" LaTeX-env-args ["argument"] 1)
    '("biography" LaTeX-env-args ["argument"] 1))
   (LaTeX-add-bibliographies
    "ref.bib"))
 :latex)

