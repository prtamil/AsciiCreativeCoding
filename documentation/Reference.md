# Reference — Consolidated Bibliography

Every program in this repo cites the papers, books, and articles it draws on
in a `REFERENCES` block in its source header. This file collects those
citations into one place, grouped by domain so each section doubles as a
reading list for that family of demos.

A handful of foundational works recur across many domains — Perlin's noise
papers, Bresenham's line algorithm, *Numerical Recipes*, Foley & van Dam's
*Computer Graphics*, the NCURSES HOWTO, Quílez's SDF articles, the
Floyd-Steinberg dither — and are listed under each domain that uses them
rather than factored out, so a section stands on its own.

Citations are deduplicated within each section. For the per-program mapping of
which demo uses which idea, read the `REFERENCES` block in the individual `.c`
file (see [DEMOS.md](../DEMOS.md) for the file index).

---

## Fluid Dynamics & PDEs  (`fluid/`)

- Alder, B. J. & Wainwright, T. E. (1959) — "Studies in Molecular Dynamics. I. General Method", *J. Chem. Phys.* 31(2), 459-466.
- Arakawa, A. & Lamb, V. R. (1977) — "Computational Design of the Basic Dynamical Processes of the UCLA General Circulation Model", *Methods Comput. Phys.* 17, 173-265.
- Batchelor, G. K. (1967) — *An Introduction to Fluid Dynamics*, Cambridge UP.
- Bourke, P. (1997) — "Character representation of grayscale images", paulbourke.net/dataformats/asciiart.
- Bridson, R. (2008) — *Fluid Simulation for Computer Graphics*, CRC Press.
- Bridson, R.; Hourihan, J. & Nordenstam, M. (2007) — "Curl-Noise for Procedural Fluid Flow", *SIGGRAPH 2007*.
- Charney, J. G.; Fjørtoft, R. & von Neumann, J. (1950) — "Numerical Integration of the Barotropic Vorticity Equation", *Tellus* 2, 237-254.
- Courant, R.; Friedrichs, K. & Lewy, H. (1928) — "Über die partiellen Differenzengleichungen der mathematischen Physik", *Math. Ann.* 100, 32-74.
- d'Humières, D.; Lallemand, P. & Frisch, U. (1986) — "Lattice gas models for 3D hydrodynamics", *Europhys. Lett.* 2, 291-297.
- Fedkiw, R.; Stam, J. & Jensen, H. W. (2001) — "Visual Simulation of Smoke", *SIGGRAPH 2001*.
- Foster, N. & Metaxas, D. (1996) — "Realistic Animation of Liquids", *GMIP* 58.
- Frisch, U.; Hasslacher, B. & Pomeau, Y. (1986) — "Lattice-gas automata for the Navier-Stokes equation", *Phys. Rev. Lett.* 56(14), 1505-1508.
- Gingold, R. A. & Monaghan, J. J. (1977) — "Smoothed Particle Hydrodynamics: theory and application to non-spherical stars", *Mon. Not. R. Astron. Soc.* 181, 375-389.
- Gray, P. & Scott, S. K. (1984) — "Autocatalytic Reactions in the Isothermal CSTR: Oscillations and Instabilities", *Chem. Eng. Sci.* 39(6), 1087-1097.
- Hardy, J.; Pomeau, Y. & de Pazzis, O. (1973) — "Time evolution of a two-dimensional model system. I", *J. Math. Phys.* 14, 1746-1759.
- Helmholtz, H. (1858) — "On Integrals of the Hydrodynamical Equations which Express Vortex-Motion", *Phil. Mag.* 33.
- Hénon, M. (1987) — "Viscosity of a lattice gas", *Complex Systems* 1, 763-789.
- Kass, M. & Miller, G. (1990) — "Rapid, Stable Fluid Dynamics for Computer Graphics", *SIGGRAPH '90*.
- LeVeque, R. J. (2002) — *Finite Volume Methods for Hyperbolic Problems*, CUP.
- LeVeque, R. J. (2007) — *Finite Difference Methods for Ordinary and Partial Differential Equations*, SIAM.
- Lucy, L. B. (1977) — "A numerical approach to the testing of the fission hypothesis", *Astron. J.* 82, 1013-1024.
- Mandelbrot, B. B. (1982) — *The Fractal Geometry of Nature*, W. H. Freeman.
- Max, N. (1995) — "Optical Models for Direct Volume Rendering", *IEEE TVCG* 1(2).
- Monaghan, J. J. (1992) — "Smoothed Particle Hydrodynamics", *Annu. Rev. Astron. Astrophys.* 30, 543-574.
- Monaghan, J. J. (2005) — "Smoothed Particle Hydrodynamics", *Rep. Prog. Phys.* 68, 1703-1759.
- Müller, M.; Charypar, D. & Gross, M. (2003) — "Particle-Based Fluid Simulation for Interactive Applications", *ACM SIGGRAPH/Eurographics SCA*.
- Nguyen, D. Q.; Fedkiw, R. & Jensen, H. W. (2002) — "Physically Based Modeling and Animation of Fire", *SIGGRAPH 2002*.
- Pearson, J. E. (1993) — "Complex Patterns in a Simple System", *Science* 261(5118), 189-192.
- Perlin, K. (1985) — "An Image Synthesizer", *SIGGRAPH 1985*, 287-296.
- Perlin, K. (2002) — "Improving noise", *SIGGRAPH 2002*.
- Quílez, I. — "Volumetric raymarching", iquilezles.org/articles/raymarchingvolumes.
- Reynolds, C. (1999) — "Steering Behaviors for Autonomous Characters", *Game Developers Conference*.
- Saad, Y. (2003) — *Iterative Methods for Sparse Linear Systems*, 2nd ed., SIAM.
- Saffman, P. G. (1992) — *Vortex Dynamics*, Cambridge UP.
- Saint-Venant, A. J. C. B. de (1871) — "Théorie du mouvement non permanent des eaux", *C. R. Acad. Sci. Paris* 73.
- Stam, J. (1999) — "Stable Fluids", *SIGGRAPH '99 Proc.*, 121-128.
- Stam, J. (2003) — "Real-Time Fluid Dynamics for Games", *GDC*.
- Stoker, J. J. (1957) — *Water Waves: The Mathematical Theory with Applications*, Wiley.
- Strikwerda, J. C. (2004) — *Finite Difference Schemes and Partial Differential Equations*, 2nd ed., SIAM.
- Turing, A. M. (1952) — "The Chemical Basis of Morphogenesis", *Phil. Trans. R. Soc. B* 237(641), 37-72.
- Voss, R. F. (1985) — "Random Fractal Forgeries", in *Fundamental Algorithms for Computer Graphics*, Springer.
- Vreugdenhil, C. B. (1994) — *Numerical Methods for Shallow-Water Flow*, Kluwer.
- Wolf-Gladrow, D. A. (2000) — *Lattice-Gas Cellular Automata and Lattice Boltzmann Models*, Springer LNM 1725.
- Yee, K. S. (1966) — "Numerical solution of initial boundary value problems involving Maxwell's equations in isotropic media", *IEEE Trans. Antennas Propag.* AP-14, 302-307.

## Physics & Numerical Methods  (`physics/`)

- Aarseth, S. J. (2003) — *Gravitational N-Body Simulations: Tools and Algorithms*, Cambridge UP.
- Ashbaugh, M. S.; Chicone, C. C. & Cushman, R. H. (1991) — "The Twisting Tennis Racket", *J. Dyn. Diff. Eq.* 3(1), 67-85.
- Baumgarte, J. (1972) — "Stabilization of constraints and integrals of motion in dynamical systems", *Comput. Methods Appl. Mech. Eng.* 1(1), 1-16.
- Bender, J.; Müller, M. & Macklin, M. (2017) — "A Survey on Position-Based Simulation Methods in Computer Graphics", *Computer Graphics Forum* 36(6).
- Berg, R. E. & Marshall, T. S. (1991) — "Pendulum waves: A lecture demonstration", *Am. J. Phys.* 59(2), 186-187.
- Bhatnagar, P. L.; Gross, E. P. & Krook, M. (1954) — "A Model for Collision Processes in Gases", *Phys. Rev.* 94, 511-525.
- Bilbao, S. (2009) — *Numerical Sound Synthesis: Finite Difference Schemes in Musical Acoustics*, Wiley.
- Binney, J. & Tremaine, S. (2008) — *Galactic Dynamics*, 2nd ed., Princeton UP.
- Birdsall, C. K. & Langdon, A. B. (2004) — *Plasma Physics via Computer Simulation*, IOP/CRC.
- Blevins, R. D. (2001) — *Formulas for Natural Frequency and Mode Shape*, Krieger.
- Brandt, A. (1977) — "Multi-Level Adaptive Solutions to Boundary-Value Problems", *Math. Comp.* 31(138), 333-390.
- Bresenham, J. E. (1965) — "Algorithm for computer control of a digital plotter", *IBM Systems Journal* 4(1), 25-30.
- Briggs, Henson & McCormick (2000) — *A Multigrid Tutorial*, 2nd ed., SIAM.
- Catto, E. (2005-2009) — "Iterative Dynamics with Temporal Coherence" & "Modeling and Solving Constraints", *GDC* / Box2D.
- Chenciner, A. & Montgomery, R. (2000) — "A remarkable periodic solution of the three-body problem in the case of equal masses", *Annals of Mathematics* 152(3), 881-901.
- Chladni, E. F. F. (1787) — *Entdeckungen über die Theorie des Klanges*, Leipzig.
- Crank, J. & Nicolson, P. (1947) — "A practical method for numerical evaluation of solutions of PDEs of the heat-conduction type", *Proc. Camb. Phil. Soc.* 43(1), 50-67.
- Diebel, J. (2006) — "Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors", Stanford tech report.
- Ericson, C. (2005) — *Real-Time Collision Detection*, Morgan Kaufmann.
- Feynman, R. P.; Leighton, R. B. & Sands, M. (1963) — *The Feynman Lectures on Physics*, Vol. I, Addison-Wesley.
- FitzHugh, R. (1961) — "Impulses and Physiological States in Theoretical Models of Nerve Membrane", *Biophys. J.* 1(6), 445-466.
- Foley, J. D.; van Dam, A.; Feiner, S. K. & Hughes, J. F. (2013) — *Computer Graphics: Principles and Practice*, 3rd ed., Addison-Wesley.
- Gere, J. M. & Goodno, B. J. (2018) — *Mechanics of Materials*, 9th ed., Cengage.
- Glaser, D. A. (1952) — "Some Effects of Ionizing Radiation on the Formation of Bubbles in Liquids", *Phys. Rev.* 87, 665.
- Goldstein, H.; Poole, C. P. & Safko, J. L. (2002) — *Classical Mechanics*, 3rd ed., Addison-Wesley.
- Graff, K. F. (1991) — *Wave Motion in Elastic Solids*, Dover.
- Griffiths, D. J. (2017/2008/2018) — *Introduction to Electrodynamics* / *Elementary Particles* / *Quantum Mechanics*.
- Hairer, E.; Lubich, C. & Wanner, G. (2006) — *Geometric Numerical Integration*, 2nd ed., Springer.
- Hestenes, M. R. & Stiefel, E. (1952) — "Methods of Conjugate Gradients for Solving Linear Systems", *J. Res. NBS* 49(6), 409-436.
- Heywood, J. B. (2018) — *Internal Combustion Engine Fundamentals*, 2nd ed., McGraw-Hill.
- Hockney, R. W. & Eastwood, J. W. (1988) — *Computer Simulation Using Particles*, IOP.
- Hodgkin, A. L. & Huxley, A. F. (1952) — "A quantitative description of membrane current…", *J. Physiol.* 117.
- Hooke, R. (1678) — *De Potentia Restitutiva, or of Spring*, Royal Society of London.
- Ising, E. (1925) — "Beitrag zur Theorie des Ferromagnetismus", *Z. Phys.* 31(1), 253-258.
- Jackson, J. D. (1998) — *Classical Electrodynamics*, 3rd ed., Wiley.
- Jakobsen, T. (2001) — "Advanced Character Physics", *GDC*.
- Kirchhoff, G. (1850) — "Über das Gleichgewicht und die Bewegung einer elastischen Scheibe", *J. Reine Angew. Math.* 40, 51-88.
- Krüger, T. et al. (2017) — *The Lattice Boltzmann Method: Principles and Practice*, Springer.
- Lagrange, J.-L. (1772/1788) — "Essai sur le problème des trois corps" / *Mécanique analytique*.
- Landau, D. P. & Binder, K. (2014) — *A Guide to Monte Carlo Simulations in Statistical Physics*, 4th ed., Cambridge.
- Landau, L. D. & Lifshitz, E. M. (1976) — *Mechanics*, Vol. 1, 3rd ed., Butterworth-Heinemann.
- Leissa, A. W. (1969) — *Vibration of Plates*, NASA SP-160.
- Lorenz, E. N. (1963) — "Deterministic Nonperiodic Flow", *J. Atmos. Sci.* 20(2), 130-141.
- Lubachevsky, B. D. (1991) — "How to Simulate Billiards and Similar Systems", *J. Comput. Phys.* 94(2), 255-283.
- Lynch, P. (2002) — "The Swinging Spring: A Simple Model of Atmospheric Balance", *Large-Scale Atmosphere-Ocean Dynamics II*, CUP.
- Metropolis, N. et al. (1953) — "Equation of State Calculations by Fast Computing Machines", *J. Chem. Phys.* 21(6), 1087-1092.
- Misner, C. W.; Thorne, K. S. & Wheeler, J. A. (1973) — *Gravitation*, W. H. Freeman.
- Mirtich, B. (1996) — *Impulse-Based Dynamic Simulation of Rigid Body Systems*, PhD thesis, UC Berkeley.
- Moreland, K. (2009) — "Diverging Color Maps for Scientific Visualization", *Proc. ISVC*, 92-103.
- Morse, P. M. & Ingard, K. U. (1968) — *Theoretical Acoustics*, McGraw-Hill/Princeton.
- Müller, M.; Heidelberger, B.; Hennix, M. & Ratcliff, J. (2007) — "Position Based Dynamics", *J. Vis. Comm. Image Repr.* 18(2), 109-118.
- Nagumo, J.; Arimoto, S. & Yoshizawa, S. (1962) — "An Active Pulse Transmission Line Simulating Nerve Axon", *Proc. IRE* 50(10), 2061-2070.
- Onsager, L. (1944) — "Crystal Statistics. I. A Two-Dimensional Model with an Order-Disorder Transition", *Phys. Rev.* 65, 117-149.
- Press, W. H. et al. (2007) — *Numerical Recipes*, 3rd ed., Cambridge UP.
- Provot, X. (1995) — "Deformation Constraints in a Mass-Spring Model to Describe Rigid Cloth Behaviour", *Graphics Interface '95*, 147-154.
- Qian, Y. H.; d'Humières, D. & Lallemand, P. (1992) — "Lattice BGK Models for Navier-Stokes Equation", *Europhys. Lett.* 17(6), 479-484.
- Rayleigh, J. W. S. (1877) — *The Theory of Sound*, Macmillan.
- Reeves, W. T. (1983) — "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects", *ACM TOG* 2(2), 91-108.
- Reuleaux, F. (1876) — *The Kinematics of Machinery*, Macmillan.
- Shewchuk, J. R. (1994) — "An Introduction to the Conjugate Gradient Method Without the Agonizing Pain", CMU CS-94-125.
- Shinbrot, T. et al. (1992) — "Chaos in a double pendulum", *Am. J. Phys.* 60(6), 491-499.
- Sparrow, C. (1982) — *The Lorenz Equations: Bifurcations, Chaos, and Strange Attractors*, Springer.
- Strauss, W. A. (2008) — *Partial Differential Equations: An Introduction*, 2nd ed., Wiley.
- Strogatz, S. H. (2014) — *Nonlinear Dynamics and Chaos*, 2nd ed., Westview.
- Succi, S. (2001) — *The Lattice Boltzmann Equation for Fluid Dynamics and Beyond*, Oxford UP.
- Taflove, A. & Hagness, S. C. (2005) — *Computational Electrodynamics: The FDTD Method*, 3rd ed., Artech House.
- Taylor, J. R. (2005) — *Classical Mechanics*, University Science Books.
- Thomas, L. H. (1949) — "Elliptic Problems in Linear Difference Equations over a Network", Watson Sci. Comput. Lab.
- Thom, A. (1933) — "The Flow Past Circular Cylinders at Low Speeds", *Proc. R. Soc. A* 141, 651-669.
- Thorne, K. S. (2014) — *The Science of Interstellar*, W. W. Norton.
- Trefethen, L. N. & Bau, D. (1997) — *Numerical Linear Algebra*, SIAM.
- Tucker, W. (1999) — "The Lorenz attractor exists", *C. R. Acad. Sci. Paris* 328(12), 1197-1202.
- Tyson, J. J. & Keener, J. P. (1988) — "Singular Perturbation Theory of Travelling Waves in Excitable Media", *Physica D* 32.
- von Kármán, T. (1911) — "Über den Mechanismus des Widerstandes…", *Nachr. Ges. Wiss. Göttingen*.
- Witkin, A. & Baraff, D. (1997/1998) — "Physically Based Modeling" / "Large Steps in Cloth Simulation", *SIGGRAPH*.
- Young, W. C.; Budynas, R. G. & Sadegh, A. M. (2011) — *Roark's Formulas for Stress and Strain*, 8th ed., McGraw-Hill.

## Chaos & Dynamical Systems  (`procedural/chaos/`)

- Arnold, V. I. (1989) — *Mathematical Methods of Classical Mechanics*, 2nd ed., Springer.
- Cvitanović, P. (ed.) (1989) — *Universality in Chaos*, 2nd ed., Adam Hilger.
- Devaney, R. L. (2003) — *An Introduction to Chaotic Dynamical Systems*, 2nd ed., Westview.
- Duffing, G. (1918) — *Erzwungene Schwingungen bei veränderlicher Eigenfrequenz*, Vieweg.
- Eckmann, J.-P. & Ruelle, D. (1985) — "Ergodic theory of chaos and strange attractors", *Rev. Mod. Phys.* 57(3), 617-656.
- Feigenbaum, M. J. (1978) — "Quantitative universality for a class of nonlinear transformations", *J. Stat. Phys.* 19(1), 25-52.
- Fiedler, G. (2004) — "Fix Your Timestep!", gafferongames.com/post/fix_your_timestep.
- Grebogi, C.; Ott, E. & Yorke, J. A. (1983) — "Fractal Basin Boundaries", *Physica D* 7.
- Greene, J. M. (1979) — "A method for determining a stochastic transition", *J. Math. Phys.* 20, 1183-1201.
- Guckenheimer, J. & Holmes, P. J. (1983) — *Nonlinear Oscillations, Dynamical Systems, and Bifurcations of Vector Fields*, Springer.
- Hénon, M. (1976) — "A two-dimensional mapping with a strange attractor", *Commun. Math. Phys.* 50(1), 69-77.
- Hénon, M. & Heiles, C. (1964) — "The applicability of the third integral of motion", *Astron. J.* 69, 73-79.
- Hirsch, M. W.; Smale, S. & Devaney, R. L. (2013) — *Differential Equations, Dynamical Systems, and an Introduction to Chaos*, 3rd ed., Academic Press.
- Holmes, P. J. (1979) — "A nonlinear oscillator with a strange attractor", *Phil. Trans. R. Soc. A* 292, 419-448.
- Letellier, C.; Dutertre, P. & Maheu, B. (1995) — "Unstable periodic orbits and templates of the Rössler system", *Chaos* 5(1), 271-282.
- Li, T.-Y. & Yorke, J. A. (1975) — "Period three implies chaos", *Am. Math. Monthly* 82.
- Lichtenberg, A. J. & Lieberman, M. A. (1992) — *Regular and Chaotic Dynamics*, 2nd ed., Springer.
- Lorenz, E. N. (1963) — "Deterministic Nonperiodic Flow", *J. Atmos. Sci.* 20(2), 130-141.
- Lorenz, E. N. (1972) — "Predictability: Does the flap of a butterfly's wings in Brazil set off a tornado in Texas?", AAAS.
- May, R. M. (1976) — "Simple mathematical models with very complicated dynamics", *Nature* 261, 459-467.
- Moon, F. C. (1992) — *Chaotic and Fractal Dynamics*, Wiley.
- Ott, E. (2002) — *Chaos in Dynamical Systems*, 2nd ed., Cambridge UP.
- Poincaré, H. (1892) — *Les Méthodes Nouvelles de la Mécanique Céleste*, Vol. III.
- Rössler, O. E. (1976) — "An equation for continuous chaos", *Phys. Lett. A* 57(5), 397-398.
- Sharkovskii, A. N. (1964) — "Coexistence of cycles of a continuous map of a line into itself", *Ukrainian Math. J.* 16.
- Sprott, J. C. (1993) — *Strange Attractors: Creating Patterns in Chaos*, M&T Books.
- Sussman, G. J. & Wisdom, J. (2014) — *Structure and Interpretation of Classical Mechanics*, 2nd ed., MIT Press.
- Tabor, M. (1989) — *Chaos and Integrability in Nonlinear Dynamics*, Wiley.
- Thompson, J. M. T. & Stewart, H. B. (2002) — *Nonlinear Dynamics and Chaos*, 2nd ed., Wiley.
- Ueda, Y. (1980) — "Steady motions exhibited by Duffing's equation", in *New Approaches to Nonlinear Problems in Dynamics* (Holmes ed.).
- Wolf, A. et al. (1985) — "Determining Lyapunov exponents from a time series", *Physica D* 16(3), 285-317.
- Yoshida, H. (1990) — "Construction of higher order symplectic integrators", *Phys. Lett. A* 150.

## Noise & Vector Fields  (`procedural/fields/`)

- Akenine-Möller, T.; Haines, E. & Hoffman, N. (2018) — *Real-Time Rendering*, 4th ed., CRC Press.
- Aurenhammer, F. (1991) — "Voronoi diagrams: a survey of a fundamental geometric data structure", *ACM Comput. Surv.* 23(3).
- Bourke, P. — "Character representation of grey scale images" / "Fractal terrain" / "Polygonising a scalar field", paulbourke.net.
- Bridson, R.; Houriham, J. & Nordenstam, M. (2007) — "Curl-noise for procedural fluid flow", *SIGGRAPH*.
- Ebert, D. S. et al. (2003) — *Texturing and Modeling: A Procedural Approach*, 3rd ed., Morgan Kaufmann.
- Felzenszwalb, P. & Huttenlocher, D. (2012) — "Distance Transforms of Sampled Functions", *Theory of Computing* 8(19).
- Foley, J. D. et al. (1995) — *Computer Graphics: Principles and Practice*, 2nd ed., Addison-Wesley.
- Fournier, A.; Fussell, D. & Carpenter, L. (1982) — "Computer rendering of stochastic models", *CACM* 25(6), 371-384.
- Gonzalez Vivo, P. & Lowe, J. — *The Book of Shaders*, ch. 13, thebookofshaders.com/13.
- Gray, P. & Scott, S. K. (1983) — "Autocatalytic reactions in the isothermal, continuous stirred tank reactor", *Chem. Eng. Sci.* 38(1), 29-43.
- Green, C. (2007) — "Improved Alpha-Tested Magnification for Vector Textures and Special Effects", *SIGGRAPH* (Valve).
- Gustavson, S. (2005) — "Simplex Noise Demystified", weber.itn.liu.se/~stegu/simplexnoise.
- Helman, J. & Hesselink, L. (1991) — "Representation and Display of Vector Field Topology in Fluid Flows", *IEEE CG&A* 11(3), 36-46.
- Lorensen, W. E. & Cline, H. E. (1987) — "Marching Cubes: A High Resolution 3D Surface Construction Algorithm", *SIGGRAPH '87*.
- Lotka, A. J. (1925) — *Elements of Physical Biology*.
- Maple, C. (2003) — "Geometric design and space planning using the marching squares and marching cube algorithms", *IV'03*.
- Marsaglia, G. (2003) — "Xorshift RNGs", *J. Stat. Software* 8(14).
- Murray, J. D. (2003) — *Mathematical Biology II: Spatial Models and Biomedical Applications*, 3rd ed., Springer.
- Newman, T. S. & Yi, H. (2006) — "A survey of the marching cubes algorithm", *Computers & Graphics* 30(5).
- Nielson, G. M. & Hamann, B. (1991) — "The Asymptotic Decider: Resolving the Ambiguity in Marching Cubes", *IEEE Visualization '91*.
- Peitgen, H.-O. & Saupe, D. (eds.) (1988) — *The Science of Fractal Images*, Springer.
- Perlin, K. (1985) — "An Image Synthesizer", *SIGGRAPH '85*, 287-296.
- Perlin, K. (2001/2002) — "Noise hardware" (course notes) / "Improving Noise", *SIGGRAPH*.
- Quílez, I. — "2D distance functions" / "Domain Warping" / "fbm" / "Palettes" / "Smooth minimum", iquilezles.org.
- Rong, G. & Tan, T-S. (2006) — "Jump Flooding in GPU with Applications to Voronoi Diagram and Distance Transform", *I3D'06*.
- Schroeder, W.; Martin, K. & Lorensen, B. (2006) — *The Visualization Toolkit*, 4th ed., Kitware.
- Turing, A. M. (1952) — "The Chemical Basis of Morphogenesis", *Phil. Trans. R. Soc. B* 237, 37-72.
- Volterra, V. (1926) — "Variazioni e fluttuazioni del numero d'individui in specie animali conviventi".
- Voss, R. F. (1985) — "Random fractal forgeries", in *Fundamental Algorithms for Computer Graphics*, Springer.

## Fractals & IFS  (`procedural/fractals/`)

- Abelson, H. & diSessa, A. (1980) — *Turtle Geometry*, MIT Press.
- Allouche, J.-P. & Shallit, J. (2003) — *Automatic Sequences*, Cambridge.
- Barnsley, M. F. (1993) — *Fractals Everywhere*, 2nd ed., Academic Press.
- Barnsley, M. F. & Demko, S. (1985) — "Iterated Function Systems and the Global Construction of Fractals", *Proc. R. Soc. Lond. A* 399, 243-275.
- Brooks, R. & Matelski, J. P. (1981) — "The Dynamics of 2-Generator Subgroups of PSL(2,C)", *Riemann Surfaces & Related Topics*, Annals of Math. Studies 97, 65-71.
- Cayley, A. (1879) — "The Newton-Fourier Imaginary Problem", *Amer. J. Math.* 2(1), 97.
- Davis, C. & Knuth, D. E. (1970) — "Number Representations and Dragon Curves, I & II", *J. Recreational Mathematics* 3, 66-81, 133-149.
- Douady, A. & Hubbard, J. H. (1982/1984-85) — "Itération des polynômes quadratiques complexes" / "Étude dynamique des polynômes complexes" (Orsay Notes).
- Draves, S. & Reckase, E. (2003) — "The Fractal Flame Algorithm", flam3.com.
- Demko, S.; Hodges, L. & Naylor, B. (1985) — "Construction of Fractal Objects with Iterated Function Systems", *SIGGRAPH '85*, 271-278.
- Falconer, K. (2003) — *Fractal Geometry: Mathematical Foundations and Applications*, 2nd ed., Wiley.
- Feigenbaum, M. J. (1978) — "Quantitative Universality for a Class of Nonlinear Transformations", *J. Stat. Phys.* 19(1), 25-52.
- Graham, Lagarias, Mallows, Wilks & Yan (2003) — "Apollonian Circle Packings: Number Theory", *J. Number Theory* 100, 1-45.
- Green, M. (1993) — "The Buddhabrot Technique", superliminal.com/fractals/bbrot.
- Hilbert, D. (1891) — "Ueber die stetige Abbildung einer Linie auf ein Flächenstück", *Math. Annalen* 38, 459-460.
- Hutchinson, J. E. (1981) — "Fractals and Self-Similarity", *Indiana Univ. Math. J.* 30(5), 713-747.
- Julia, G. (1918) — "Mémoire sur l'itération des fonctions rationnelles", *J. Math. Pures Appl.* 8, 47-245.
- Kim, T. & Lin, M. C. (2004) — "Physically Based Animation and Rendering of Lightning", *Pacific Graphics 2004*.
- Lagarias, Mallows & Wilks (2002) — "Beyond the Descartes Circle Theorem", *Am. Math. Monthly* 109, 338-361.
- Libbrecht, K. G. (2005) — "The physics of snow crystals", *Rep. Prog. Phys.*
- Lindenmayer, A. (1968) — "Mathematical Models for Cellular Interaction in Development, I & II", *J. Theor. Biol.* 18, 280-315.
- Mandelbrot, B. B. (1980/1982) — "Fractal Aspects of the Iteration of z → λz(1−z)", *Annals NY Acad. Sci.* 357 / *The Fractal Geometry of Nature*, Freeman.
- Michelitsch, M. & Rössler, O. E. (1992) — "The 'Burning Ship' and its quasi-Julia sets", *Computers & Graphics* 16(4), 435-438.
- Milnor, J. (2006) — *Dynamics in One Complex Variable*, 3rd ed., Princeton.
- Mumford, Series & Wright (2002) — *Indra's Pearls: The Vision of Felix Klein*, Cambridge UP.
- Nakaya, U. (1954) — *Snow Crystals: Natural and Artificial*, Harvard UP.
- Niemeyer, L.; Pietronero, L. & Wiesmann, H. J. (1984) — "Fractal Dimension of Dielectric Breakdown", *Phys. Rev. Lett.* 52(12), 1033.
- Peitgen, H.-O. & Richter, P. H. (1986) — *The Beauty of Fractals*, Springer.
- Peitgen, Jürgens & Saupe (1992/2004) — *Chaos and Fractals: New Frontiers of Science*, Springer.
- Pineda, J. (1988) — "A Parallel Algorithm for Polygon Rasterization", *SIGGRAPH '88*, 17-20.
- Prusinkiewicz, P. (1986) — "Graphical Applications of L-systems", *Graphics Interface '86*, 247-253.
- Prusinkiewicz, P. & Lindenmayer, A. (1990) — *The Algorithmic Beauty of Plants*, Springer.
- Reed, T. & Wyvill, B. (1994) — "Visual Simulation of Lightning", *SIGGRAPH '94*, 359-364.
- Rosenfeld, A. & Pfaltz, J. L. (1966) — "Sequential Operations in Digital Picture Processing", *J. ACM* 13(4), 471.
- Sierpiński, W. (1915) — "Sur une courbe dont tout point est un point de ramification", *C. R. Acad. Sci. Paris* 160, 302-305.
- Soddy, F. (1936) — "The Kiss Precise", *Nature* 137, 1021.
- Vepstas, L. (2004) — "Renormalizing the Mandelbrot Escape".
- von Koch, H. (1904) — "Sur une courbe continue sans tangente…", *Arkiv för Matematik* 1, 681-704.
- Witten, T. A. & Sander, L. M. (1981) — "Diffusion-Limited Aggregation, a Kinetic Critical Phenomenon", *Phys. Rev. Lett.* 47(19), 1400.

## Cellular Automata & Generative  (`procedural/generational/`)

- Adamatzky, A. (ed.) (2010) — *Game of Life Cellular Automata*, Springer.
- Bak, Tang & Wiesenfeld (1987) — "Self-Organized Criticality: an explanation of 1/f noise", *Phys. Rev. Lett.* 59, 381.
- Bays, C. (2005) — "A Note on the Game of Life in Hexagonal and Pentagonal Tessellations", *Complex Systems* 15.
- Berlekamp, Conway & Guy (2004) — *Winning Ways for Your Mathematical Plays*, vol. 4, 2nd ed., AK Peters.
- Bridson, R. (2007) — "Fast Poisson Disk Sampling in Arbitrary Dimensions", *SIGGRAPH sketches*.
- Buck, Jamis (2014/2015) — "Rooms and Mazes: A Procedural Dungeon Generator" / *Mazes for Programmers*, Pragmatic Bookshelf.
- Chan, Bert (2019/2020) — "Lenia: Biology of Artificial Life", *Complex Systems* 28(3), arXiv:1812.05433 / "Lenia and Expanded Universe", *ALIFE 2020*, arXiv:2005.03742.
- Cook, M. (2004) — "Universality in Elementary Cellular Automata", *Complex Systems* 15(1), 1-40.
- Cormen, Leiserson, Rivest & Stein — *Introduction to Algorithms* (CLRS).
- Davidenko et al. (1992) — "Stationary and drifting spiral waves of excitation in isolated cardiac muscle", *Nature* 355, 349.
- Dewdney, A. K. (1984) — "Computer Recreations: …Wa-Tor", *Scientific American* 251(6).
- Dhar, D. (1990) — "Self-organised critical state of sandpile automaton models", *Phys. Rev. Lett.* 64, 1613.
- Eden, M. (1961) — "A two-dimensional growth process", *Proc. 4th Berkeley Symp. Math. Stat. Prob.*
- Gajardo, A.; Moreira, A. & Goles, E. (2002) — "Complexity of Langton's ant", *Discrete Appl. Math.* 117.
- Gardner, M. (1970) — "Mathematical Games: …Conway's new solitaire game 'life'", *Scientific American* 223(4).
- Gerhardt, M.; Schuster, H. & Tyson, J. (1990) — "A cellular automaton model of excitable media", *Physica D* 46, 392.
- Greenberg, J. & Hastings, S. (1978) — "Spatial Patterns for Discrete Models of Diffusion in Excitable Media", *SIAM J. Appl. Math.* 34(3), 515.
- Grimm, V. & Railsback, S. F. (2005) — *Individual-based Modeling and Ecology*, Princeton.
- Gumin, M. (2016) — "WaveFunctionCollapse", github.com/mxgmn/WaveFunctionCollapse.
- Johnson, D.; Yannakakis, M. & Togelius, J. (2010) — "Cellular Automata for Real-time Generation of Infinite Cave Levels", *PCGames'10*, ACM.
- Karth & Smith (2017) — "WaveFunctionCollapse is Constraint Solving in the Wild", *FDG 2017*.
- Knuth, D. — *TAOCP* vol. 2, §3.4.2 (Fisher-Yates shuffle).
- Langton, C. G. (1986) — "Studying artificial life with cellular automata", *Physica D* 22.
- Levine, D. & Propp, J. (2010) — "What is … a Sandpile?", *AMS Notices* 57(8).
- Mackworth, A. K. (1977) — "Consistency in Networks of Relations" (AC-3 arc consistency).
- Meakin, P. (1998) — *Fractals, Scaling and Growth Far from Equilibrium*, Cambridge UP.
- Merrell, P. (2007) — "Example-Based Model Synthesis", *I3D 2007*.
- Rafler, S. (2011) — "SmoothLife: Generalization of Conway's Game of Life to a continuous domain", arXiv:1111.1567.
- Shaker, N.; Togelius, J. & Nelson, M. J. (2016) — *Procedural Content Generation in Games*, Springer; pcgbook.com.
- Spitzer, F. (1964) — *Principles of Random Walk*, 2nd ed., Springer.
- Vicsek, T. (1992) — *Fractal Growth Phenomena*, 2nd ed., World Scientific.
- Wiener, N. & Rosenblueth, A. (1946) — "The mathematical formulation of the problem of conduction of impulses … in cardiac muscle", *Arch. Inst. Cardiol. Mex.* 16, 205.
- Wilson, D. B. (1996) — "Generating random spanning trees more quickly than the cover time", *STOC*.
- Winfree, A. T. (2001) — *The Geometry of Biological Time*, 2nd ed., Springer.
- Wolfram, S. (1983/1984/2002) — "Statistical mechanics of cellular automata", *Rev. Mod. Phys.* 55 / "Universality and complexity in cellular automata", *Physica D* 10 / *A New Kind of Science*, Wolfram Media.
- Patel, Amit (Red Blob Games) — Interactive graph-search and grid visualisations, redblobgames.com.

## Tilings & Aperiodic Patterns  (`procedural/patterns/`)

- Allouche, J.-P. & Shallit, J. (2003) — *Automatic Sequences*, Cambridge.
- Austin, D. (2005) — "Penrose Tiles Talk Across Miles", *AMS Feature Column*.
- Aurenhammer, F. (1991) — "Voronoi Diagrams: A Survey of a Fundamental Geometric Data Structure", *ACM Comput. Surv.* 23(3).
- de Bruijn, N. G. (1981) — "Algebraic theory of Penrose's non-periodic tilings of the plane, I & II", *Indag. Math.* 43, 39-66.
- Ebert, D. et al. (2003) — *Texturing & Modeling: A Procedural Approach*, 3rd ed.
- Falconer, K. (2014) — *Fractal Geometry: Mathematical Foundations and Applications*, 3rd ed., Wiley.
- Gardner, M. (1977) — "Extraordinary nonperiodic tiling that enriches the theory of tiles", *Scientific American* 236(1), 110-121.
- Grünbaum, B. & Shephard, G. C. (1987) — *Tilings and Patterns*, W. H. Freeman, ch. 10.
- Moreland, K. (2009) — "Diverging Color Maps for Scientific Visualization", *Proc. ISVC 2009*.
- Okabe, A.; Boots, B.; Sugihara, K. & Chiu, S. N. (2000) — *Spatial Tessellations: Concepts and Applications of Voronoi Diagrams*, 2nd ed.
- Patel, Amit (Red Blob Games) — "Hexagonal Grids", redblobgames.com/grids/hexagons.
- Penrose, R. (1974) — "The role of aesthetics in pure and applied mathematical research", *Bull. Inst. Math. Appl.* 10, 266-271.
- Prusinkiewicz, P. & Lindenmayer, A. (1990) — *The Algorithmic Beauty of Plants*, Springer.
- Senechal, M. (1995) — *Quasicrystals and Geometry*, Cambridge UP.
- Wang, H. (1961) — "Proving theorems by pattern recognition", *Bell System Technical J.* 40, 1-41.
- Wolfram, S. (2002) — *A New Kind of Science*, Wolfram Media.

## Procedural Worldgen  (`procedural/worldgen/`)

- Beyer, D. (2015) — Hydraulic erosion tutorial implementation.
- Ebert, D. et al. (2003) — *Texturing & Modeling: A Procedural Approach*, 3rd ed.
- Fournier, A.; Fussell, D. & Carpenter, L. (1982) — "Computer Rendering of Stochastic Models", *CACM* 25(6).
- Imhof, E. (1982) — *Cartographic Relief Presentation* (hypsometric tinting).
- Lague, Sebastian — Hydraulic erosion procedural tutorials.
- Lorensen, W. & Cline, H. (1987) — "Marching Cubes", *SIGGRAPH*.
- Mandelbrot, B. B. (1982) — *The Fractal Geometry of Nature*, Freeman.
- Miller, G. S. P. (1986) — "The Definition and Rendering of Terrain Maps", *SIGGRAPH*.
- Musgrave, F. K.; Kolb, C. & Mace, R. (1989) — "The Synthesis and Rendering of Eroded Fractal Terrains", *SIGGRAPH*.
- Perlin, K. (1985/2002) — "An Image Synthesizer" / "Improving Noise", *SIGGRAPH*.
- Shaker, N.; Togelius, J. & Nelson, M. J. (2016) — *Procedural Content Generation in Games*, Springer; pcgbook.com.

## Grids, Tessellation & Coordinate Systems  (`grids/`)

- Bowyer, A. (1981) — "Computing Dirichlet Tessellations", *The Computer Journal* 24(2), 162-166.
- Conway, J. H. & Radin, C. (1998) — "Quaquaversal Tilings and Rotations", *Inventiones Mathematicae* 132.
- Coxeter, H. S. M. (1973) — *Regular Polytopes*, 3rd ed., Dover (§4.6).
- de Berg, M. et al. — *Computational Geometry: Algorithms and Applications*, 3rd ed. (§9).
- Hatcher, A. (2002) — *Algebraic Topology*, Cambridge UP (§2.1, barycentric subdivision).
- Hutchinson, J. E. (1981) — "Fractals and Self-Similarity", *Indiana Univ. Math. J.* 30(5).
- Livio, M. (2002) — *The Golden Ratio*, ch. 5.
- Lowe, D. (2004) — "Distinctive Image Features from Scale-Invariant Keypoints", *IJCV* 60(2), 91-110.
- Mandelbrot, B. B. (1982) — *The Fractal Geometry of Nature*, Freeman (§6).
- Nystrom, R. — *Game Programming Patterns* (object pool), gameprogrammingpatterns.com/object-pool.html.
- Penrose, R. (1979) — "Pentaplexity", *Mathematical Intelligencer* 2.
- Prusinkiewicz, P. & Lindenmayer, A. (1990) — *The Algorithmic Beauty of Plants*, ch. 4 (phyllotaxis).
- Radin, C. (1994) — "The Pinwheel Tilings of the Plane", *Annals of Mathematics* 139(3).
- Patel, Amit (Red Blob Games) — "Hexagonal Grids" (cube coordinates, distances, rounding, rings), redblobgames.com/grids/hexagons.
- Robinson, R. M. (1971) — "Undecidability and Nonperiodicity for Tilings of the Plane", *Inventiones Mathematicae* 12.
- Sadun, L. (2008) — *Topology of Tiling Spaces*, AMS (§1).
- Schwartz, E. L. (1980) — "Spatial Mapping in the Primate Sensory Projection…", *Biol. Cybernetics* 37(4), 199-208 (log-polar).
- Senechal, M. (1995) — *Quasicrystals and Geometry*, Cambridge UP (§7).
- Shewchuk, J. R. (1996) — "Robust Adaptive Floating-Point Geometric Predicates".
- Sierpiński, W. (1915) — original gasket construction, *C. R. Acad. Sci. Paris* 160.
- Vogel, H. (1979) — "A better way to construct the sunflower head", *Mathematical Biosciences* 44(3-4), 179-189.
- Watson, D. F. (1981) — "Computing the n-dimensional Delaunay tessellation with application to Voronoi polytopes", *The Computer Journal* 24(2).
- Reference geometry articles (Wikipedia / MathWorld): triangular, trihexagonal, rhombille, tetrakis-square, and pinwheel tilings; Archimedean tilings; Cartesian / polar / log-polar coordinates; Archimedean, logarithmic, and golden spirals; rotation matrices; isometric projection; pixel aspect ratio.

## Rasterization & 3D Rendering  (`raster/`)

- Akenine-Möller et al. (2019) — *Real-Time Rendering*, 4th ed. (§16.2.4).
- Bavoil & Sainz (2009) — "Multi-Layer Dual-Resolution Screen-Space Ambient Occlusion", *SIGGRAPH '09*.
- Blinn, J. F. (1977) — "Models of Light Reflection for Computer Synthesised Pictures", *SIGGRAPH '77*.
- Bourke, P. — "ASCII art" character ramp, paulbourke.net/dataformats/asciiart.
- Cook, R. L. (1984) — "Shade Trees", *SIGGRAPH '84*.
- Eddington, A. S. (1926) — *The Internal Constitution of the Stars*, Cambridge UP.
- Foley, van Dam, Feiner & Hughes — *Computer Graphics: Principles and Practice*, 3rd ed.
- Helland, T. — "How to Convert Temperature (K) to RGB", tannerhelland.com/2012/09/18.
- Hubbard & Douady (1985) — "Étude dynamique des polynômes complexes", *Pub. Math. Orsay '85*.
- James & O'Rorke (2004) — "Real-Time Glow", *GPU Gems*.
- LearnOpenGL — "Bloom" / "SSAO" / "Shadow Mapping" tutorials, learnopengl.com.
- Lorensen & Cline (1987) — "Marching Cubes", *SIGGRAPH '87*.
- Mitchell et al. (2007) — "Real-Time Rendering Tricks for Ambient Occlusion and Edge Detection", *GDC 2007*.
- Möller, T. (2000) — "Fast Triangle Rasterization by Interpolating Edge Functions", *Game Programming Gems*.
- Mittring, M. (2007) — "Finding Next Gen — CryEngine 2", *SIGGRAPH '07* course notes.
- Newman & Sproull (1979) — *Principles of Interactive Computer Graphics*, 2nd ed. (Ch. 22).
- Phong, B. T. (1975) — "Illumination for Computer Generated Pictures", *CACM* 18(6).
- Perlin, K. (1985) — "An Image Synthesizer", *SIGGRAPH '85*, 287-296.
- Quílez, I. — "Distance Functions" / "Distance Estimators for Implicit Surfaces" / "Mandelbulb DE", iquilezles.org.
- Reinhard et al. (2002) — "Photographic Tone Reproduction for Digital Images", *SIGGRAPH '02*.
- Saito & Takahashi (1990) — "Comprehensible Rendering of 3-D Shapes", *SIGGRAPH '90*.
- Sloane, A. (2011) — "Donut math: how donut.c works", a1k0n.net/2011/07/20/donut-math.html.
- Sobel & Feldman (1968) — "A 3×3 Isotropic Gradient Operator for Image Processing", *SAIL*.
- White & Nylander (2009) — "Hypercomplex Fractals".
- Williams, L. (1978) — "Casting Curved Shadows on Curved Surfaces", *SIGGRAPH '78*.

## Raymarching & Signed Distance Fields  (`raymarcher/`)

- Barnsley, M. (1988) — *Fractals Everywhere*.
- Blinn, J. F. (1982) — "A Generalization of Algebraic Surface Drawing", *ACM TOG* 1(3), 235-256.
- Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for the Antialiased Ray Tracing of Implicit Surfaces", *Visual Computer* 12(10), 527-545.
- Knighty (2010) — "Kaleidoscopic (escape time) IFS", Fractal Forums thread.
- Phong, B. T. (1975) — "Illumination for Computer Generated Pictures", *CACM* 18(6), 311-317.
- Quílez, I. — "Distance Functions" / "Smooth Minimum" / "Distance Estimators for Implicit Surfaces", iquilezles.org.
- White & Nylander (2009) — "Mandelbulb", skytopia.com/project/fractal/mandelbulb.html.

## Ray Tracing & Global Illumination  (`raytracing/`)

- Beason, K. — "smallpt: Global Illumination in 99 lines of C++", kevinbeason.com/smallpt.
- Cerezo, E. et al. (2005) — "A Survey on Participating Media Rendering Techniques", *The Visual Computer*.
- Eddington (1926) — *The Internal Constitution of the Stars*, §8 (limb darkening).
- Goral et al. (1984) — "Modeling the Interaction of Light Between Diffuse Surfaces", *SIGGRAPH '84*.
- Hanrahan, P. (1983) — "Ray Tracing Algebraic Surfaces", *SIGGRAPH '83*.
- Hapke, B. — *Theory of Reflectance and Emittance Spectroscopy*, 2nd ed.
- Hearn & Baker — *Computer Graphics with OpenGL*, 4th ed., ch. 10.
- Helland, T. — "How to Convert Temperature (K) to RGB", tannerhelland.com.
- Henyey, L. G. & Greenstein, J. L. (1941) — "Diffuse radiation in the galaxy", *Astrophysical Journal*.
- Kajiya, J. T. (1986) — "The Rendering Equation", *SIGGRAPH '86*.
- Kay & Kajiya (1986) — "Ray Tracing Complex Scenes", *SIGGRAPH '86*.
- Malley, T. (1988) — "A Shading Method for Computer Generated Images", MS thesis, Univ. of Utah.
- Pharr, Jakob & Humphreys (2016) — *Physically Based Rendering: From Theory to Implementation*, 4th ed., pbr-book.org.
- Press et al. — *Numerical Recipes in C*, 2nd ed., §5.6.
- Quílez, I. — "Intersectors" (sphere / capsule / box / torus), iquilezles.org/articles/intersectors.
- Reinhard et al. (2002) — "Photographic Tone Reproduction for Digital Images", *SIGGRAPH '02*.
- Schlick, C. (1994) — "An Inexpensive BRDF Model for Physically-based Rendering", *Comput. Graph. Forum* 13(3).
- Shirley, P. — "Ray Tracing in One Weekend", raytracing.github.io.
- Shirley & Marschner — *Fundamentals of Computer Graphics*, 4th ed., ch. 4.
- Veach, E. (1997) — "Robust Monte Carlo Methods for Light Transport Simulation", PhD thesis, Stanford.
- Whitted, T. (1980) — "An Improved Illumination Model for Shaded Display", *CACM* 23(6).
- Reference astronomy/optics articles: Saturn's rings, Cassini Division, solar corona, K-corona, Thomson scattering, solar prominence, Hα line, Bailey's beads, eclipse penumbra, implicit torus equation.

## Artistic & Demoscene  (`artistic/`)

- Akasofu, S.-I. (2007) — *Exploring the Secrets of the Aurora*, 2nd ed., Springer.
- Alberts et al. (2014) — *Molecular Biology of the Cell*, 6th ed.
- Bourke, P. (1997) — "Character representation of grey-scale images" / "Parametric equations of a helix / circle", paulbourke.net.
- Chamberlain, J. W. (1961) — *Physics of the Aurora and Airglow*, Academic Press / AGU.
- Coxeter, H. S. M. (1973) — *Regular Polytopes* (star polygons, symmetry groups).
- Critchlow, K. (1976) — *Islamic Patterns: An Analytical and Cosmological Approach*.
- Eather, R. H. (1980) — *Majestic Lights: The Aurora in Science, History, and the Arts*, AGU.
- Ebert et al. (2003) — *Texturing & Modeling: A Procedural Approach*, 3rd ed.
- Finch, M. (2004) — "Effective Water Simulation from Physical Models", *GPU Gems* ch. 1.
- Foley, van Dam, Feiner & Hughes (1990) — *Computer Graphics: Principles and Practice*, 2nd ed.
- Grünbaum & Shephard (1987) — *Tilings and Patterns*.
- Jarzynski, M. & Olano, M. (2020) — "Hash Functions for GPU Rendering", *JCGT* 9(3).
- Kulaichev, A. P. (1984) — "Sriyantra and its mathematical properties", *Indian J. History of Science*.
- Lindenmayer, A. (1968) — "Mathematical models for cellular interactions in development", *J. Theor. Biol.* 18.
- Lissaman, P. B. S. & Shollenberger, C. A. (1970) — "Formation Flight of Birds", *Science* 168.
- Perlin, K. (1985) — "An Image Synthesizer", *SIGGRAPH*.
- Prusinkiewicz, P. & Lindenmayer, A. (1990) — *The Algorithmic Beauty of Plants*, Springer.
- Reynolds, C. W. (1987/1999) — "Flocks, Herds, and Schools", *SIGGRAPH '87* / "Steering Behaviors for Autonomous Characters", *GDC*.
- Saenger, W. (1984) — *Principles of Nucleic Acid Structure*, Springer.
- Sen, D. & Gilbert, W. (1988) — "Formation of parallel four-stranded complexes by guanine-rich motifs", *Nature* 334.
- Wang, A. H. J. et al. (1979) — "Molecular structure of a left-handed double helical DNA fragment at atomic resolution", *Nature* 282.
- Watson, J. D. & Crick, F. H. C. (1953) — "Molecular Structure of Nucleic Acids", *Nature* 171.
- Williams, R. (2001) — *The Animator's Survival Kit*, Faber.

## Particle Systems  (`particle_systems/`)

- Akenine-Möller, T.; Haines, E. & Hoffman, N. (2018) — *Real-Time Rendering*, 4th ed.
- Bagnold, R. A. (1941) — *The Physics of Blown Sand and Desert Dunes*, Dover reprint.
- Bak, P.; Tang, C. & Wiesenfeld, K. (1987) — "Self-Organized Criticality: An Explanation of 1/f Noise", *Phys. Rev. Lett.* 59(4).
- Blinn, J. F. (1982) — "A Generalization of Algebraic Surface Drawing", *ACM TOG* 1.
- Bloomenthal, I. et al. (1997) — *Introduction to Implicit Surfaces*, Morgan Kaufmann.
- Bourg, D. M. & Bywalec, B. (2013) — *Physics for Game Developers*, 2nd ed., O'Reilly.
- Bridson, R.; Houriham, J. & Nordenstam, M. (2007) — "Curl-Noise for Procedural Fluid Flow", *ACM TOG* 26(3).
- Dabiri, J. O. et al. (2005) — "Flow patterns generated by oblate medusan jellyfish", *J. Exp. Biol.* 208.
- Di Battista, G. et al. (1999) — *Graph Drawing: Algorithms for the Visualization of Graphs*.
- Duran, J. (2000) — *Sands, Powders, and Grains*, Springer.
- Floyd, R. W. & Steinberg, L. (1976) — "An Adaptive Algorithm for Spatial Greyscale", *Proc. SID* 17(2).
- Gamma, Helm, Johnson & Vlissides (1994) — *Design Patterns*, Addison-Wesley.
- Garg, K. & Nayar, S. K. (2007) — "Vision and Rain", *IJCV* 75(1).
- Gemmell, B. J. et al. (2013) — "Passive energy recapture in jellyfish", *PNAS* 110(44).
- Imhof, E. (1975) — "Positioning Names on Maps", *The American Cartographer*.
- Jaeger, H. M.; Nagel, S. R. & Behringer, R. P. (1996) — "Granular solids, liquids, and gases", *Rev. Mod. Phys.* 68(4).
- Knuth, D. E. (1997) — *The Art of Computer Programming, Vol. 2*, 3rd ed.
- Kuhn, H. (1955) — "The Hungarian Method for the Assignment Problem", *Naval Research Logistics Quarterly* 2.
- Lin, C. C. & Shu, F. H. (1964) — "On the Spiral Structure of Disk Galaxies", *Astrophysical Journal*.
- Lowe, D. (2004) — "Critically Damped Ease-In/Ease-Out Smoothing", *Game Programming Gems* 4.
- Lu, P. J. & Steinhardt, P. J. (2007) — "Decagonal and Quasi-Crystalline Tilings in Medieval Islamic Architecture", *Science* 315.
- Millington, I. (2007) — *Game Physics Engine Development*, 2nd ed.
- Musgrave, F. K.; Kolb, C. E. & Mace, R. S. (1989) — "The Synthesis and Rendering of Eroded Fractal Terrains", *SIGGRAPH*.
- Newman, M. E. J. (2010) — *Networks: An Introduction*, Oxford UP.
- Newell, Newell & Sancha (1972) — "A solution to the hidden surface problem", *Proc. ACM Annual Conference*.
- Nguyen, D. Q.; Fedkiw, R. & Jensen, H. W. (2002) — "Physically Based Modeling and Animation of Fire", *SIGGRAPH*.
- Nystrom, R. (2014) — *Game Programming Patterns*, Genever Benning.
- Ovenden, M. (2007) — *Transit Maps of the World*.
- Penner, R. (2002) — "Robert Penner's Easing Functions".
- Perlin, K. (2002) — "Improving Noise", *ACM SIGGRAPH*.
- Press, W. H. et al. (2007) — *Numerical Recipes*, 3rd ed.
- Rankine, W. J. M. (1858) — *A Manual of Applied Mechanics* (§625, vortex).
- Reeves, W. T. (1983) — "Particle Systems — A Technique for Modeling a Class of Fuzzy Objects", *ACM TOG* 2(2).
- Reeves, W. T. & Blau, R. (1985) — "Approximate and Probabilistic Algorithms for Shading and Rendering Structured Particle Systems", *SIGGRAPH '85*.
- Rubin, V. C. & Ford, W. K. (1970) — "Rotation of the Andromeda Nebula…", *Astrophysical Journal*.
- Sims, K. (1990) — "Particle Animation and Rendering Using Data Parallel Computation", *SIGGRAPH '90*.
- Spencer, G. et al. (1995) — "Physically-Based Glare Effects for Digital Images", *SIGGRAPH*.
- Stam, J. (1999/2003) — "Stable Fluids", *SIGGRAPH '99* / "Real-Time Fluid Dynamics for Games", *GDC*.
- Stevens, S. S. (1957) — "On the psychophysical law", *Psychological Review* 64(3).
- Toomre, A. (1977) — "Theories of Spiral Structure", *Annu. Rev. Astron. Astrophys.*
- Tritton, D. J. (1988) — *Physical Fluid Dynamics*, Oxford.
- Uhlenbeck, G. E. & Ornstein, L. S. (1930) — "On the Theory of the Brownian Motion", *Phys. Rev.* 36.
- Vandevenne, L. — "Lode's Computer Graphics Tutorial — Plasma", lodev.org.
- Witkin, A. & Heckbert, P. S. (1994) — "Using Particles to Sample and Control Implicit Surfaces", *SIGGRAPH*.
- Wyvill, G.; McPheeters, C. & Wyvill, B. (1986) — "Data Structure for Soft Objects", *The Visual Computer* 2.

## Matrix Rain  (`matrix_rain/`)

- Bresenham, J. E. (1965) — "Algorithm for computer control of a digital plotter", *IBM Systems Journal* 4(1).
- Foley, J. D. et al. (2013) — *Computer Graphics: Principles and Practice*, 3rd ed.
- Padala, P. — *NCURSES Programming HOWTO*, The Linux Documentation Project.
- Prusinkiewicz, P. & Lindenmayer, A. (1990) — *The Algorithmic Beauty of Plants*, Springer.

## Animation & Kinematics  (`animation/`, `robots/`)

- Aristidou, A. & Lasenby, J. (2011) — "FABRIK: a fast, iterative solver for the inverse kinematics problem", *Graphical Models* 73(5), 243-260.
- Aristidou, A.; Chrysanthou, Y. & Lasenby, J. (2016) — "Extending FABRIK with model constraints", *Computer Animation and Virtual Worlds* 27(1).
- Blickhan, R. (1989) — "The spring-mass model for running and hopping", *J. Biomech.* 22(11-12), 1217-1227.
- Boulic, R.; Magnenat-Thalmann, N. & Thalmann, D. (1990) — "A global human walking model with real-time kinematic personification", *The Visual Computer* 6, 344-358.
- Buss, S. R. (2004) — "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares", UCSD course notes.
- Craig, J. J. (2005) — *Introduction to Robotics: Mechanics and Control*, 3rd ed., Pearson.
- Cruse, H. (1990) — "What mechanisms coordinate leg movement in walking arthropods?", *Trends in Neurosciences* 13(1).
- Denavit, J. & Hartenberg, R. S. (1955) — "A kinematic notation for lower-pair mechanisms based on matrices", *J. Appl. Mech.* 22, 215-221.
- Dudek, G. & Jenkin, M. (2010) — *Computational Principles of Mobile Robotics*, 2nd ed., Cambridge UP.
- Full, R. J. & Tu, M. S. (1991) — "Mechanics of a rapid running insect", *J. Exp. Biol.* 156, 215-231.
- Geyer, H.; Seyfarth, A. & Blickhan, R. (2006) — "Compliant leg behaviour explains basic dynamics of walking and running", *Proc. R. Soc. B* 273(1603).
- Hecker, C. (2005) — "Behind the Screen — Verlet Physics", *Game Developer Magazine*.
- Hirose, S. (1993) — *Biologically Inspired Robots: Snake-Like Locomotors and Manipulators*, Oxford.
- Inman, V. T.; Ralston, H. J. & Todd, F. (1994) — *Human Walking*, 2nd ed., Williams & Wilkins.
- Jakobsen, T. (2001) — "Advanced Character Physics", *GDC*.
- LaValle, S. M. (2006) — *Planning Algorithms*, Cambridge UP.
- Lissajous, J. A. (1857) — "Mémoire sur l'étude optique des mouvements vibratoires", *Annales de Chimie et de Physique* 51.
- Manton, S. M. (1952) — "The evolution of arthropodan locomotory mechanisms", *J. Linn. Soc. (Zoology)* 42, 93-117.
- Müller, M.; Heidelberger, B.; Hennix, M. & Ratcliff, J. (2007) — "Position Based Dynamics", *J. Vis. Comm. Image Repr.* 18(2), 109-118.
- Murray, R. M.; Li, Z. & Sastry, S. S. (1994) — *A Mathematical Introduction to Robotic Manipulation*, CRC.
- Parent, R. (2012) — *Computer Animation: Algorithms and Techniques*, 3rd ed., Morgan Kaufmann.
- Perlin, K. (1985/1995/2002) — "An Image Synthesizer" / "Real Time Responsive Animation with Personality", *IEEE TVCG* 1(1) / "Improving Noise", *SIGGRAPH*.
- Raibert, M. H. (1986) — *Legged Robots That Balance*, MIT Press.
- Reynolds, C. W. (1999) — "Steering Behaviors for Autonomous Characters", *GDC 1999*.
- Siegwart, R.; Nourbakhsh, I. & Scaramuzza, D. (2011) — *Introduction to Autonomous Mobile Robots*, 2nd ed., MIT Press.
- Spong, M. W.; Hutchinson, S. & Vidyasagar, M. (2005) — *Robot Modeling and Control*, Wiley.
- Sumbre, G. et al. (2001) — "Control of octopus arm extension by a peripheral motor program", *Science* 293, 1845-1848.
- Verlet, L. (1967) — "Computer Experiments on Classical Fluids. I", *Phys. Rev.* 159(1), 98-103.
- Wang, L.-C. T. & Chen, C. C. (1991) — "A combined optimization method for solving the inverse kinematics problem", *IEEE Trans. Robotics Automat.* 7(4).
- Winter, D. A. (2009) — *Biomechanics and Motor Control of Human Movement*, 4th ed., Wiley.
- Åström, K. J. & Hägglund, T. (2006) — *Advanced PID Control*, ISA.
- Åström, K. J. & Murray, R. M. (2008) — *Feedback Systems: An Introduction for Scientists and Engineers*, Princeton.

## Flocking & Emergence  (`flocking/`)

- Adamatzky, A. (2010) — *Physarum Machines: Computers from Slime Mould*, World Scientific.
- Cavagna, A. et al. (2010) — "Scale-free correlations in starling flocks", *PNAS* 107(26), 11865-11870.
- Couzin, I. D. et al. (2002) — "Collective memory and spatial sorting in animal groups", *J. Theor. Biol.* 218(1), 1-11.
- Grassé, P.-P. (1959) — "La reconstruction du nid et les coordinations interindividuelles…" (stigmergy), *Insectes Sociaux* 6, 41-83.
- Helbing, D. & Molnár, P. (1995) — "Social force model for pedestrian dynamics", *Phys. Rev. E* 51(5), 4282-4286.
- Helbing, D.; Farkas, I. & Vicsek, T. (2000) — "Simulating dynamical features of escape panic", *Nature* 407(6803), 487-490.
- Hildenbrandt, H.; Carere, C. & Hemelrijk, C. K. (2010) — "Self-organized aerial displays of thousands of starlings: a model", *Behav. Ecol.* 21(6), 1349-1359.
- Jones, J. (2010) — "Characteristics of pattern formation and evolution in approximations of Physarum transport networks", *Artificial Life* 16(2), 127-153.
- King, A. J. et al. (2012) — "Selfish-herd behaviour of sheep under threat", *Current Biology* 22(14), R561-R562.
- Kuhn, H. W. (1955) — "The Hungarian method for the assignment problem", *Naval Research Logistics Quarterly* 2(1-2), 83-97.
- Lanchester, F. W. (1916) — *Aircraft in Warfare: The Dawn of the Fourth Arm*.
- Lien, J.-M. et al. (2004) — "Shepherding behaviors", *IEEE ICRA*, 4159-4164.
- Millington, I. & Funge, J. (2019) — *Artificial Intelligence for Games*, 3rd ed., CRC Press.
- Nakagaki, T.; Yamada, H. & Tóth, Á. (2000) — "Maze-solving by an amoeboid organism", *Nature* 407(6803), 470.
- Reynolds, C. W. (1987) — "Flocks, Herds, and Schools: A Distributed Behavioral Model", *SIGGRAPH '87* 21(4), 25-34.
- Shiffman, D. — *The Nature of Code*, ch. 6, natureofcode.com.
- Strömbom, D. et al. (2014) — "Solving the shepherding problem: heuristics for herding autonomous, interacting agents", *J. R. Soc. Interface* 11(100), 20140719.
- Tero, A. et al. (2007/2010) — "A mathematical model for adaptive transport network in path finding by true slime mold", *J. Theor. Biol.* 244(4) / "Rules for biologically inspired adaptive network design", *Science* 327(5964), 439-442.
- Teschner, M. et al. (2003) — "Optimized spatial hashing for collision detection of deformable objects", *VMV 2003*, 47-54.
- Toner, J. & Tu, Y. (1995/1998) — "Long-Range Order in a Two-Dimensional Dynamical XY Model" / "Flocks, herds, and schools", *Phys. Rev. Lett.* 75 / *Phys. Rev. E* 58.
- Vaughan, R. et al. (2000) — "Experiments in automatic flock control", *Robotics and Autonomous Systems* 31, 109-117.
- Vicsek, T. et al. (1995) — "Novel type of phase transition in a system of self-driven particles", *Phys. Rev. Lett.* 75(6), 1226-1229.

## Algorithms & Data Structures  (`algorithms/`)

- Anderson, R. M. & May, R. M. (1991) — *Infectious Diseases of Humans: Dynamics and Control*, Oxford UP.
- Aurenhammer, F. (1991) — "Voronoi diagrams — A survey of a fundamental geometric data structure", *ACM Comput. Surv.* 23(3), 345-405.
- Barabási, A.-L. & Albert, R. (1999) — "Emergence of scaling in random networks", *Science* 286, 509-512.
- Bentley, J. L. (1975/1990) — "Multidimensional binary search trees used for associative searching", *CACM* 18(9) / "K-d trees for semidynamic point sets", *SCG '90*.
- Chan, T. M. (1996) — "Optimal output-sensitive convex hull algorithms in two and three dimensions", *Discrete & Comput. Geom.* 16(4), 361-368.
- Cormen, Leiserson, Rivest & Stein (2009) — *Introduction to Algorithms*, 3rd ed., MIT Press.
- de Berg, M. et al. (2008) — *Computational Geometry: Algorithms and Applications*, 3rd ed., Springer.
- Delaunay, B. (1934) — "Sur la sphère vide", *Bull. Acad. Sci. URSS* 7, 793-800.
- Di Battista, G. et al. (1999) — *Graph Drawing: Algorithms for the Visualization of Graphs*, Prentice Hall.
- Eades, P. (1984) — "A Heuristic for Graph Drawing", *Congressus Numerantium* 42, 149-160.
- Finkel, R. A. & Bentley, J. L. (1974) — "Quad trees: a data structure for retrieval on composite keys", *Acta Informatica* 4(1), 1-9.
- Fortune, S. (1987) — "A sweepline algorithm for Voronoi diagrams", *Algorithmica* 2, 153-174.
- Fruchterman, T. M. J. & Reingold, E. M. (1991) — "Graph drawing by force-directed placement", *Software: Practice and Experience* 21(11), 1129-1164.
- Graham, R. L. (1972) — "An efficient algorithm for determining the convex hull of a finite planar set", *Inf. Process. Lett.* 1(4), 132-133.
- Hart, P. E.; Nilsson, N. J. & Raphael, B. (1968) — "A Formal Basis for the Heuristic Determination of Minimum Cost Paths", *IEEE Trans. Syst. Sci. Cybern.* 4(2), 100-107.
- Jarvis, R. A. (1973) — "On the identification of the convex hull of a finite set of points in the plane", *Inf. Process. Lett.* 2(1), 18-21.
- Keeling, M. J. & Rohani, P. (2008) — *Modeling Infectious Diseases in Humans and Animals*, Princeton UP.
- Kermack, W. O. & McKendrick, A. G. (1927) — "A contribution to the mathematical theory of epidemics", *Proc. R. Soc. A* 115, 700-721.
- Knuth, D. E. (1997/1998) — *The Art of Computer Programming*, Vol. 1 (3rd ed.) & Vol. 3 (2nd ed.), Addison-Wesley.
- Lloyd, S. P. (1982) — "Least squares quantization in PCM", *IEEE Trans. Inf. Theory* 28(2), 129-137.
- Newman, M. E. J. (2010) — *Networks: An Introduction*, Oxford UP.
- O'Rourke, J. (1998) — *Computational Geometry in C*, 2nd ed., Cambridge.
- Samet, H. (1984/1990/2006) — "The quadtree and related hierarchical data structures", *ACM Comput. Surv.* 16(2) / *The Design and Analysis of Spatial Data Structures* / *Foundations of Multidimensional and Metric Data Structures*.
- Sedgewick, R. (1978) — "Implementing Quicksort programs", *CACM* 21(10), 847-857.
- Sedgewick, R. & Wayne, K. (2011) — *Algorithms*, 4th ed., Addison-Wesley.
- Shamos, M. I. & Hoey, D. (1975) — "Closest-point problems", *16th FOCS*, 151-162.
- Shewchuk, J. R. (1997) — "Adaptive precision floating-point arithmetic and fast robust geometric predicates", *Discrete & Comput. Geom.* 18(3), 305-363.
- Voronoi, G. (1908) — "Nouvelles applications des paramètres continus à la théorie des formes quadratiques", *J. reine angew. Math.* 134, 198-287.
- Watts, D. J. & Strogatz, S. H. (1998) — "Collective dynamics of small-world networks", *Nature* 393, 440-442.
- Williams, J. W. J. (1964) — "Algorithm 232: Heapsort", *CACM* 7(6), 347-348.

## Geometry & Curves  (`geometry/`)

- Abelson & diSessa (1981) — *Turtle Geometry: The Computer as a Medium for Exploring Mathematics*, MIT Press.
- Asano, T. et al. (1986) — "Visibility of disjoint polygons", *Algorithmica* 1(1), 49-63.
- Bowditch, N. (1815) — "On the motion of a pendulum suspended from two points", *Mem. Am. Acad. Arts Sci.* 3(2), 413-436.
- Bowyer, A. (1981) — "Computing Dirichlet tessellations", *The Computer Journal* 24(2), 162-166.
- Cundy, H. M. & Rollett, A. P. (1961) — *Mathematical Models*, Oxford UP.
- de Berg, M. et al. (2008) — *Computational Geometry: Algorithms and Applications*, 3rd ed., Springer.
- ElGindy, H. & Avis, D. (1981) — "A linear algorithm for computing the visibility polygon from a point", *J. Algorithms* 2(2), 186-197.
- Lawrence, J. D. (1972) — *A Catalog of Special Plane Curves*, Dover.
- Lee, D. T. (1983) — "Visibility of a simple polygon", *CVGIP* 22(2), 207-221.
- Lissajous, J. A. (1857) — "Mémoire sur l'étude optique des mouvements vibratoires", *Annales de Chimie et de Physique* 51, 147-231.
- Lockwood, E. H. (1961) — *A Book of Curves*, Cambridge UP.
- Maor, E. (1998) — *Trigonometric Delights*, Princeton UP.
- Maurer, P. M. (1987) — "A Rose is a Rose...", *Am. Math. Monthly* 94(7), 631-645.
- Quílez, I. (2015) — "Palettes", iquilezles.org/articles/palettes.
- Shewchuk, J. R. (1996/1997) — "Triangle: Engineering a 2D Quality Mesh Generator and Delaunay Triangulator", *LNCS 1148* / "Adaptive Precision Floating-Point Arithmetic and Fast Robust Geometric Predicates", *Discrete & Comput. Geom.* 18(3).
- Sloan, S. W. (1987) — "A fast algorithm for constructing Delaunay triangulations in the plane", *Advances in Engineering Software* 9(1), 34-55.
- Watson, D. F. (1981) — "Computing the n-dimensional Delaunay tessellation with application to Voronoi polytopes", *The Computer Journal* 24(2), 167-172.
- Whitaker, R. J. (2001) — "Harmonographs. I & II", *Am. J. Phys.* 69(2), 162-167 & 174-183.

## Signal Processing  (`signal/`)

- Cooley, J. W. & Tukey, J. W. (1965) — "An Algorithm for the Machine Calculation of Complex Fourier Series", *Mathematics of Computation* 19, 297-301.
- Oppenheim & Lim (1981) — "The importance of phase in signals", *IEEE*.
- Oppenheim, A. V. & Schafer, R. W. (2010) — *Discrete-Time Signal Processing*, 3rd ed., Pearson.
- Papoulis, A. — *Signal Analysis*, McGraw-Hill.
- Press, W. H. et al. — *Numerical Recipes in C*, ch. 12.
- Smith, S. W. — *The Scientist and Engineer's Guide to Digital Signal Processing*, ch. 12.

## AI & Learning  (`Ai/`)

- Eiben & Smith (2015) — *Introduction to Evolutionary Computing*, 2nd ed., Springer.
- Goldberg, D. E. (1989) — *Genetic Algorithms in Search, Optimization, and Machine Learning*, Addison-Wesley.
- Goodfellow, Bengio & Courville (2016) — *Deep Learning*, MIT Press.
- Holland, J. H. (1992) — *Adaptation in Natural and Artificial Systems* (orig. 1975), MIT Press.
- LeCun, Bengio & Hinton (2015) — "Deep learning", *Nature* 521, 436-444.
- Marsaglia, G. (1972) — "Choosing a point from the surface of a sphere", *Ann. Math. Stat.* 43(2), 645-646.
- Mitchell, M. (1996) — *An Introduction to Genetic Algorithms*, MIT Press.
- Nielsen, M. (2015) — *Neural Networks and Deep Learning*, neuralnetworksanddeeplearning.com.
- Olah, C. (2014) — "Neural Networks, Manifolds, and Topology", colah.github.io.
- Schmidhuber, J. (2015) — "Deep Learning in Neural Networks: An Overview", *Neural Networks* 61.
- Shiffman, D. — *The Nature of Code*, ch. 9, natureofcode.com.
- Smilkov, Carter et al. (2017) — "TensorFlow Playground", playground.tensorflow.org.

## Terminal & Framework  (`turtle/`, `ncurses_basics/`)

- Bresenham, J. E. (1965) — "Algorithm for computer control of a digital plotter", *IBM Systems Journal* 4(1), 25-30.
- Fiedler, G. — "Fix Your Timestep!", gafferongames.com.
- Newman & Sproull (1979) — *Principles of Interactive Computer Graphics*, 2nd ed., McGraw-Hill.
- Padala, P. — *NCURSES Programming HOWTO*, The Linux Documentation Project.
- Papert, S. (1980) — *Mindstorms: Children, Computers, and Powerful Ideas*, Basic Books.

---

*Generated by scanning the `REFERENCES` block of every source file. Some
foundational works recur across sections; per-file attribution lives in each
`.c` header.*
