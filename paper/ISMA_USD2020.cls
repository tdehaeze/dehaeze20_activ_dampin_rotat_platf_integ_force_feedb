%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ISMA_USD2020.cls, v1, 2020/01/07						 %
% Developed by Simon Vandemaele							 %
% 														 %
% This class file enables authors to prepare papers		 %
% for submission to the ISMA2020 and USD2020 conference. %
%														 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


\NeedsTeXFormat{LaTeX2e}   %
\ProvidesClass{ISMA_USD2020}%[2020/01/07, v1]

%Load the Times New Roman font
	\RequirePackage{times}     
	
%The class is adapted form the article class
\LoadClass[a4paper, 11pt]{article}	
	

%Loading of required packages
	\usepackage{mfirstuc}
	\usepackage[utf8]{inputenc}
	\usepackage{longtable,tabularx}
	\usepackage{amsmath,amssymb,siunitx,graphicx,epstopdf,pdfpages,bm,relsize}
	\usepackage{pdfpages}
	\usepackage{epsfig}
	\usepackage{epstopdf}
	\usepackage{eurosym}

%Setting of the margins and line spacing 
	\usepackage[top=30mm, bottom=22mm, left=22mm, right=22mm]{geometry}
	\setlength{\paperheight}     {297mm} %
	\setlength{\paperwidth}      {210mm} %
	\setlength{\textwidth}       {166mm} %
	\setlength{\hoffset}       {-3.04mm} %
	\setlength{\oddsidemargin}     {0mm} %
	\setlength{\voffset}           {4.725mm} %	
	\setlength{\topmargin}         {0mm} %
	\setlength{\headheight}        {0mm} %
	\setlength{\headsep}           {0mm} %
	\setlength{\textheight}      {245mm} %
	\setlength{\parindent}         {0mm} %
	\setlength{\parskip} {5pt plus 2pt minus 1pt} %
	\pagestyle{empty}      
	\usepackage{setspace}
	\setstretch{0.95}              %

%Loading and general set-up hyperref package
	\usepackage[pdftex,hidelinks]{hyperref}
	\hypersetup{a4paper = true,
		pdfpagemode = None,
		pdfstartview  = FitH
		}

%Defining title block
	\makeatletter
	\renewcommand\maketitle{\par
		\begingroup
		\renewcommand\thefootnote{\@fnsymbol\c@footnote}%
		\def\@makefnmark{\rlap{\@textsuperscript{\normalfont\@thefnmark}}}%
		\long\def\@makefntext##1{\parindent 1em\noindent
			\hb@xt@1.8em{%
				\hss\@textsuperscript{\normalfont\@thefnmark}}##1}%
		\@maketitle%
		\thispagestyle{empty}\@thanks
		\endgroup
		\setcounter{footnote}{0}%
		\global\let\thanks\relax
		\global\let\maketitle\relax
		\global\let\@maketitle\relax
		\global\let\@thanks\@empty
		\global\let\@author\@empty
		\global\let\@affil\@empty
		\global\let\@date\@empty
		\global\let\@title\@empty
		\global\let\title\relax
		\global\let\author\relax
		\global\let\affil\relax
		\global\let\date\relax
		\global\let\and\relax
	}
	\def\@maketitle{%
		\newpage
		\null
		\let \footnote \thanks
		\noindent{\LARGE\sffamily\bfseries\@title \par
		}%
		\vskip 22.5pt%
		\noindent{\normalsize \@author %\par
		}%
		\vskip 31pt%
		{\Large \@date %\par
		}%
		\vskip 30pt}
	\renewcommand{\abstract}[1]
	{\date{\flushleft\parbox[t]{\textwidth}{%
				\textbf{\sffamily\Large Abstract}\\ \normalsize #1}
		}%
	}

%Defining the author and affiliation blocks
	\RequirePackage[blocks]{authblk}
	\renewcommand{\Authfont}{\bfseries}
	\setlength{\affilsep}{-0.5pt}
	
	%%%Sep affiliation
	\newcommand{\NewLineAffil}{ \protect \\ }
	\newcommand{\NewAffil}{\vspace{12.5pt}}
	
	%%% changing commands to add necessary spaces in titling
	\renewcommand\AB@authnote[1]{~\textsuperscript{\Affilfont#1}}
	\renewcommand\AB@affilnote[1]{\textsuperscript{\Affilfont#1}~}
	
	% Defining to only use comma's between authors 
	\@namedef{@sep2}{\Authsep} %
	
	
	\renewcommand\author[2][]%
	{\ifnewaffil\addtocounter{affil}{1}%
		\edef\AB@thenote{\arabic{affil}}\fi
		\if\relax#1\relax\def\AB@note{\AB@thenote}\else\def\AB@note{#1}%
		\setcounter{Maxaffil}{0}\fi
		\ifnum\value{authors}>1\relax
		\@namedef{@sep\number\c@authors}{\Authsep}\fi
		\addtocounter{authors}{1}%
		\begingroup
		\let\protect\@unexpandable@protect \let\and\AB@pand
		\def\thanks{\protect\thanks}\def\footnote{\protect\footnote}%
		\@temptokena=\expandafter{\AB@authors}%
		{\def\\{\protect\\[\@affilsep]\protect\Affilfont
				\protect\AB@resetsep }%
			\xdef\AB@author{\AB@blk@and#2}%
			\ifnewaffil\gdef\AB@las{}\gdef\AB@lasx{\protect\Authand}\gdef\AB@as{}%
			\xdef\AB@authors{\the\@temptokena\AB@blk@and}%
			\else
			\xdef\AB@authors{\the\@temptokena\AB@as\AB@au@str}%
			\global\let\AB@las\AB@lasx\gdef\AB@lasx{\protect\Authands}%
			\gdef\AB@as{\Authsep}%
			\fi
			\gdef\AB@au@str{#2}}%
		\@temptokena=\expandafter{\AB@authlist}%
		\let\\=\authorcr
		\xdef\AB@authlist{\the\@temptokena
			\protect\@nameuse{@sep\number\c@authors}%
			\protect\Authfont#2\AB@authnote{\AB@note}}%
		\endgroup
		\ifnum\value{authors}>2\relax
		\@namedef{@sep\number\c@authors}{\Authsep}\fi %%% changed to remove And originally \Authands
		\newaffilfalse
	}
	
	\renewcommand\affil[2][]%
	{\newaffiltrue\let\AB@blk@and\AB@pand
		\if\relax#1\relax\def\AB@note{\AB@thenote}\else\def\AB@note{#1}%
		\setcounter{Maxaffil}{0}\fi
		\begingroup
		\let\protect\@unexpandable@protect
		\def\thanks{\protect\thanks}\def\footnote{\protect\footnote}%
		\@temptokena=\expandafter{\AB@authors}%
		{\def\\{\protect\\ \protect\Affilfont}\xdef\AB@temp{#2}}%
		\xdef\AB@authors{\the\@temptokena\AB@las\AB@au@str
			\protect\\[\affilsep]\protect\Affilfont\AB@temp}%
		\gdef\AB@las{}\gdef\AB@au@str{}%
		{\def\\{, \ignorespaces}\xdef\AB@temp{#2}}%
		\@temptokena=\expandafter{\AB@affillist}%
		\xdef\AB@affillist{\the\@temptokena \AB@affilsep
			\AB@affilnote{\AB@note}\protect\Affilfont\AB@temp}%
		\endgroup
		\let\AB@affilsep\AB@affilsepx 
	}

%Defining section headers
	\renewcommand\thesection      {\@arabic\c@section}
	\renewcommand\thesubsection   {\thesection.\@arabic\c@subsection}
	\renewcommand\section{\@startsection {section}{1}{\z@}%
		{-3.5ex \@plus -1ex \@minus -.2ex}%
		{2.3ex \@plus.2ex}%
		{\normalfont\Large\bfseries\sffamily}}
	\renewcommand\subsection{\@startsection{subsection}{2}{\z@}%
		{-3.25ex\@plus -1ex \@minus -.2ex}%
		{1.5ex \@plus .2ex}%
		{\normalfont\large\bfseries\sffamily}}
	\renewcommand\subsubsection{\@startsection{subsubsection}{3}{\z@}%
		{-3.25ex\@plus -1ex \@minus -.2ex}%
		{1.5ex \@plus .2ex}%
		{\normalfont\normalsize\bfseries\sffamily}}
	\makeatother

%Caption settings
	\usepackage{floatrow}
	\floatsetup[table]{capposition=top}
	
	\renewcommand{\tablename}{Table}
	\renewcommand{\figurename}{Figure}

%Enumeration settings
	\usepackage{enumitem}
	\setitemize{itemsep=5pt,topsep=0pt,parsep=0pt,partopsep=0pt}

%Settings referencing style to IEEE
	\bibliographystyle{ieeetran}

