Operationnal Space Optimal Controller
============

### Task Formulation 
<img src="http://www.sciweavers.org/tex2img.php?eq=%5Cvect%7Bg%7D%5Cleft%28%5Cboldsymbol%7B%5Ctau%7D%2C%5Cvect%7B%5Cddot%7BX%7D%7D%5Ec%5Cright%29%20%3D%20%20%5Cvect%7B%5Cddot%7BX%7D%7D%5Ec%20-%20%5Cvect%7B%5Cddot%7BX%7D%7D&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\vect{g}\left(\boldsymbol{\tau},\vect{\ddot{X}}^c\right) =  \vect{\ddot{X}}^c - \vect{\ddot{X}}" width="227" height="50" />

<img src="http://www.sciweavers.org/tex2img.php?eq=%5Cvect%7Bg%7D%5Cleft%28%5Cboldsymbol%7B%5Ctau%7D%2C%5Cvect%7B%5Cddot%7BX%7D%7D%5Ec%5Cright%29%20%3D%20%20%5Cvect%7B%5Cddot%7BX%7D%7D%5Ec%20-%20%5Cleft%28J%28%5Cvect%7Bq%7D%29%20M%28%5Cvect%7Bq%7D%29%5E%7B-1%7D%20%5Cleft%28%5Cboldsymbol%7B%5Ctau%7D%20-%20%5Cvect%7Bb%7D%28%5Cvect%7Bq%7D%2C%5Cvect%7B%5Cdot%7Bq%7D%7D%29%20%5Cright%29%20%2B%20%5Cdot%7BJ%7D%28%5Cvect%7Bq%7D%29%20%5Cvect%7B%5Cdot%7Bq%7D%7D%5Cright%29&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\vect{g}\left(\boldsymbol{\tau},\vect{\ddot{X}}^c\right) =  \vect{\ddot{X}}^c - \left(J(\vect{q}) M(\vect{q})^{-1} \left(\boldsymbol{\tau} - \vect{b}(\vect{q},\vect{\dot{q}}) \right) + \dot{J}(\vect{q}) \vect{\dot{q}}\right)" width="583" height="50" />

With : 

<img src="http://www.sciweavers.org/tex2img.php?eq=%5Cddot%7BX%7D%5Ec%20%3D%20%5Cddot%7BX%7D%5Ed%20%2B%20K_%7Bp%7D%28X%5Ed%20-%20X%29%20%2B%20K_%7Bd%7D%28%5Cdot%7BX%7D%5Ed%20-%20%5Cdot%7BX%7D%29&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\ddot{X}^c = \ddot{X}^d + K_{p}(X^d - X) + K_{d}(\dot{X}^d - \dot{X})" width="429" height="33" />

##### Constraints

<img src="http://www.sciweavers.org/tex2img.php?eq=%5Cleft%5C%7B%5Cbegin%7Barray%7D%7Blcl%7D%5Cvect%7Bq%7D_%7Bmin%7D%20%5Cleq%20%5Cvect%7Bq%7D_%7B%7Ck%2B1%7D%20%5Cleq%20%5Cvect%7Bq%7D_%7Bmax%7D%2C%20%5C%5C%5Cvect%7B%5Cdot%7Bq%7D%7D_%7Bmin%7D%20%5Cleq%20%5Cvect%7B%5Cdot%7Bq%7D%7D_%7B%7Ck%2B1%7D%20%5Cleq%20%5Cvect%7B%5Cdot%7Bq%7D%7D_%7Bmax%7D%2C%20%5C%5C%5Cboldsymbol%7B%5Ctau%7D_%7Bmin%7D%20%5Cleq%20%5Cboldsymbol%7B%5Ctau%7D_%7B%7Ck%7D%20%5Cleq%20%5Cboldsymbol%7B%5Ctau%7D_%7Bmax%7D.%5Cend%7Barray%7D%5Cright&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\left\{\begin{array}{lcl}\vect{q}_{min} \leq \vect{q}_{|k+1} \leq \vect{q}_{max}, \\\vect{\dot{q}}_{min} \leq \vect{\dot{q}}_{|k+1} \leq \vect{\dot{q}}_{max}, \\\boldsymbol{\tau}_{min} \leq \boldsymbol{\tau}_{|k} \leq \boldsymbol{\tau}_{max}.\end{array}\right" width="250" height="94" />


<img src="http://www.sciweavers.org/tex2img.php?eq=%5Cleft%5C%7B%5Cbegin%7Barray%7D%7Bl%7D%5Cvect%7B%5Cddot%7Bq%7D%7D_%7B%7Ck%7D%20%3D%20M%28%5Cvect%7Bq%7D_%7B%7Ck%7D%29%5E%7B-1%7D%20%5Cleft%28%5Cboldsymbol%7B%5Ctau%7D_%7B%7Ck%7D%20-%20%5Cvect%7Bb%7D%28%5Cvect%7Bq%7D_%7B%7Ck%7D%2C%5Cvect%7B%5Cdot%7Bq%7D_%7B%7Ck%7D%7D%29%5Cright%29%2C%5C%5C%5Cvect%7B%5Cdot%7Bq%7D%7D_%7B%7Ck%2B1%7D%20%20%3D%20%20%5Cvect%7B%5Cdot%7Bq%7D%7D_%7B%7Ck%7D%20%2B%20%5Cdelta%20t%20%5Cvect%7B%5Cddot%7Bq%7D%7D_%7B%7Ck%7D%2C%5C%5C%5Cvect%7Bq%7D_%7B%7Ck%2B1%7D%20%3D%20%5Cvect%7Bq%7D_%7B%7Ck%7D%20%2B%20%5Cdelta%20t%20%5Cvect%7B%5Cdot%7Bq%7D%7D_%7B%7Ck%7D%20%2B%20%5Cfrac%7B%5Cdelta%20t%5E%7B2%7D%7D%7B2%7D%20%5Cvect%7B%5Cddot%7Bq%7D%7D_%7B%7Ck%7D.%5Cend%7Barray%7D%5Cright&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\left\{\begin{array}{l}\vect{\ddot{q}}_{|k} = M(\vect{q}_{|k})^{-1} \left(\boldsymbol{\tau}_{|k} - \vect{b}(\vect{q}_{|k},\vect{\dot{q}_{|k}})\right),\\\vect{\dot{q}}_{|k+1}  =  \vect{\dot{q}}_{|k} + \delta t \vect{\ddot{q}}_{|k},\\\vect{q}_{|k+1} = \vect{q}_{|k} + \delta t \vect{\dot{q}}_{|k} + \frac{\delta t^{2}}{2} \vect{\ddot{q}}_{|k}.\end{array}\right" width="385" height="96" />


### Controller formulation
<img src="http://www.sciweavers.org/tex2img.php?eq=%5Carg_%7B%5Climits_%7B%5Cboldsymbol%7B%5Ctau%7D%7D%7D%20min%20%20%5Cleft%5C%7C%20%5Cvect%7Bg%7D%5Cleft%28%5Cboldsymbol%7B%5Ctau%7D%2C%5Cvect%7B%5Cddot%7BX%7D%7D%5Ec%5Cright%29%20%5Cright%5C%7C_%7BQ_t%7D%5E2&bc=Transparent&fc=Black&im=png&fs=18&ff=modern&edit=0" align="center" border="0" alt="\arg_{\limits_{\boldsymbol{\tau}}} min  \left\| \vect{g}\left(\boldsymbol{\tau},\vect{\ddot{X}}^c\right) \right\|_{Q_t}^2" width="258" height="56" />

# Launch in Simulation

```bash
# Start your controller in simulation
roslaunch cart_opt_ctrl run.launch sim:=true
```
# Launch on Hardware

```bash
# Start RTnet connection to KRC
rosrun lwr_scrits rtnet start
# Start your controller
roslaunch cart_opt_ctrl run.launch
```

### Reference
```
Control of robots sharing their workspace with humans: an energetic approach to safety
A. Meguenani, V. Padois and P. Bidaud
IROS, 2015.
```
Bibtex: 
```
@inproceedings{meguenani_IROS2015,
    title = {Control of robots sharing their workspace with humans: an energetic approach to safety},
    author = {Meguenani, A. and Padois, V. and Bidaud, P.},
    booktitle = {Proceedings of the IEEE/RSJ International Conference on Intelligent Robots and Systems},
    pages = {},
    doi = {},
    http = {http://hal.archives-ouvertes.fr/hal-01179822/en},
    year = {2015},
    month = Sep,
    address = {Hamburg, Germany}
}
```
