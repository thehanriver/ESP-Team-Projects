# Quest 4: Electronic Voting
Authors: Mario Han, Vivek Cherian, Hussain Valiuddin

Date: 2020-11-13
-----
## Summary

  The website contains three buttons: "All Candidates" will show the most recent votes (R,G,B) by the candidates (1,2,3,4,5,6). "Votes by Candidates" shows all the votes depending on red, green, blue and shows the counts of the votes. "Clear database" clears database and should show nothing when any of the read buttons are clicked.
  
  One problem we are trying to fix is the data doesn't load on the first click but the second click. For example, clicking All Candidates when the website is loaded won't show anything. Clicking it again will show it. This follows the same pattern when Clear database is clicked.
        
## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One |  |  1     | 
| Objective Two |  |  1     | 
| Objective Three |  |  1     | 
| Objective Four |  |  1     | 
| Objective Five |  |  1     | 
| Objective Six |  |  1     | 
| Objective Seven |  |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution |  |  5     | 
| Quality of report.md including use of graphics |  |  3     | 
| Quality of code reporting |  |  3     | 
| Quality of video presentation |  |  3     | 


## Solution Design


  For showing the votes on the webpage, we used flags with POST and GET just like the previous quest. This allowed us to "run" certain functions in index.js based on a timer. These functions would constantly check for when the flag is raised to read the database which is set at 50ms timer. The flag is raised when the button is clicked and gives enough time to load all data points to /all, /all/red, /all/green, /all/blue. This is then displayed in the main page of Quest 4.
  

## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References

-----

