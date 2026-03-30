# RSL Student Onboarding

Living reference for initializing a new student project in the `RSL_OngoingStudentProjects` shared Drive.

## Scope

When a new student starts, handle these steps:

1. Add the student to the shared spreadsheet.
2. Create the Drive project folder from the template.
3. Add the grading sheet at the outer project-folder level.
4. Request cluster permission.
5. Request compute-room access.
6. Send Fang Nan the student name and GitHub handle so the student can be added to the student GitHub group.

## Shared Resources

- Shared Drive name: `RSL_OngoingStudentProjects`
- Shared Drive id: `0AIU36ZEZMCB3Uk9PVA`
- Project spreadsheet: `List of Student Projects`
- Spreadsheet id: `1w_IP91LTrw-JgUT1bXRjrK6xoZ87yLGG4lZd0Q4qfS0`
- Main tab for active students: `Ongoing Projects`
- `2026` folder id: `151pUcb0tKCMsAw4B8d0A392Oznhqu_Sm`
- `Templates` folder id: `1bPe5jjMBciHGETsrlILS7tBlXJJSAWUX`
- Template content folder `shared`: `16Kf_-zAQUyT_SlSrI0-e80i76QvtL3nL`
- Grading-sheet template: `RSL - Student Evaluation Template`
- Grading-sheet template id: `1RbxUwJWIYBjc8yOdmber_5ASUsQGAP3qaFqfbyjAdUU`
- Kick-off doc template: `Kick-Off-Student-Projects`
- Kick-off doc template id: `1oe128vMqzn_0WGKuLXZMKgeYQ22Xj550BGSYgCXJ-fo`
- PowerPoint template file: `RSL_Template_Powerpoint.pptx`
- PowerPoint template id: `1nev1IQnBXv2dY51997_CFdFnKaNQ5U7a`

## Step 1: Add Student To Spreadsheet

Use the `Ongoing Projects` tab.

Observed layout:

- rows `1-2`: blank
- row `3`: header
- row `4+`: active projects

Important rule:

- New students must be inserted from the top, directly below the header.
- Do not append at the bottom.
- Do not use the helper append flow for this sheet. It appends to the detected table end and can land in the wrong place.

Safe procedure:

1. Insert a new row at row `4`.
2. Write the student record into `A4:V4`.
3. Verify the row appears directly under the header and before the existing active projects.

Current columns in use:

- `A`: Supervisors
- `B`: Student name
- `C`: Committee Member(s)
- `D`: Github Account
- `E`: Bitbucket Email
- `F`: Thesis Type
- `G`: Title
- `H`: Description
- `I`: Year
- `J`: Start date
- `K`: End date
- `L`: GDrive Folder
- `M`: Completed
- `N`: Report available
- `O`: Grading available
- `P`: Source files available
- `Q`: Access revoked
- `R`: Project Type (HW|SW)
- `S`: currently blank in the sheet layout
- `T`: Grade eDoz (Mariat)
- `U`: Archive Box Nr.
- `V`: Agreed RSL Policy

## Step 2: Create Drive Project Folder

Target location:

- Put the new project under the current year folder, currently `2026`.

Observed structure for existing projects:

1. Outer project folder in `/2026`
2. Grading sheet copied into the outer project folder
3. One nested student-material folder inside the outer folder
4. Template documents and numbered subfolders live inside that nested student-material folder

Naming convention for new folders:

- Outer folder: `MT - Student Name` or `MA - Student Name`
- Inner folder: `<outer folder name> Student Folder`

Template payload to copy into the inner student folder:

- `00_Administration/`
- `01_Meetings/`
- `02_Project_Management/`
- `03_Research/`
- `04_Requirements_and_Function_structure/`
- `05_Concepts/`
- `06_Evaluation/`
- `07_Design/`
- `08_Tests_and_Validation/`
- `09_Report_and_Presentation/`
- `Start_reading_here.docx`
- `Kick-Off-Student-Projects`
- `euler-cluster-guide.txt`
- `lab_organization_student_projects_student_pcs - RSL Wiki.pdf`
- `RSL_Template_Powerpoint.pptx`

Notes:

- `Kick-Off-Student-Projects` exists as a Google Doc template.
- The grading sheet belongs in the outer project folder, not inside the nested student-material folder.
- If a create/copy run is interrupted, inspect the partial folder tree and resume idempotently. Do not blindly recreate the whole folder.

## Step 3: Add Grading Sheet

1. Copy `RSL - Student Evaluation Template`.
2. Place the copy in the outer project folder.
3. Keep it outside the nested student-material folder.

Current convention:

- Keeping the default name `RSL - Student Evaluation Template` is acceptable.
- Historical folders are inconsistent, so only rename if the user explicitly asks.

## Step 4: Request Cluster Permission

Current request form:

- `https://docs.google.com/forms/d/e/1FAIpQLSd9OgeyS7iaZ6WkASOH7kNEfHMGrXIbYdBlzmoWOmLITQJmKw/viewform`

Form title:

- `Request Cluster (Euler) Access`

Required fields:

- `eth_username`
- `last_name`
- `first_name`
- `eth_email`
- `termination_date`
- `supervisors`

Optional field:

- `main_supervisor_email`

Current content expectations:

- `eth_username`: ETH username without domain
- `eth_email`: must match `@ethz.ch`
- `termination_date`: end of thesis / access end date
- `supervisors`: comma-separated names
- `main_supervisor_email`: email of the main supervisor who confirms the request

Notes:

- Tell the student to inform the supervisor first; the form text says the supervisor needs to confirm the application.
- If automating the submission, use the form's inner response IDs, not the visible question IDs.
- If automating the submission, first open a prefilled `viewform` URL, then extract fresh `partialResponse` and `fbzx` from that page before posting to `formResponse`.
- A plain `formResponse` POST with only the visible `entry.<question_id>` fields can fail silently and bounce back to the form with required-field errors.
- Inner response IDs observed for this form:
  - `eth_username`: `1648523464`
  - `last_name`: `1992375209`
  - `first_name`: `1463250709`
  - `eth_email`: `1045487484`
  - `termination_date`: `68168767`
  - `supervisors`: `2022322873`
  - `main_supervisor_email`: `534103135`
- Worked example that succeeded:
  - `eth_username`: `madina`
  - `last_name`: `Madina`
  - `first_name`: `Lorenzo`
  - `eth_email`: `madina@ethz.ch`
  - `termination_date`: `2026-09-24`
  - `supervisors`: `Lorenzo Terenzi`
  - `main_supervisor_email`: `lterenzi@ethz.ch`
- The official references linked by the form are:
  - `https://leggedrobotics.github.io/euler-cluster-guide/`
  - `https://scicomp.ethz.ch/wiki/Euler`

## Step 5: Request Compute-Room Access

Current request form:

- `https://docs.google.com/forms/d/e/1FAIpQLSdlpMApH6KQe4z096hNWFqlJyoMRb6xv8vpTc4aZwYuB8n_zw/viewform`

Form title:

- `Request a Student Place`

What this form is for:

- shared workstation / PC-room access for RSL students during a semester, bachelor thesis, or master thesis/project

Important distinction from the Euler form:

- this form is signed-in and tied to the supervisor Google account
- it explicitly records `lterenzi@leggedrobotics.com`
- the raw anonymous `viewform` / `formResponse` trick used for the Euler form is not the right path here
- `gws` cannot create Google Forms responses

Required Chrome / CDP prerequisites:

- Chrome remote debugging must be enabled in the live browser:
  - open `chrome://inspect/#remote-debugging`
  - turn the toggle on
- local CDP tool repo:
  - `/home/lorenzo/git/chrome-cdp-skill`
- local CDP wrapper:
  - `/home/lorenzo/git/chrome-cdp-skill/skills/chrome-cdp/scripts/cdp`

Recommended submission path:

1. Open the form in the already logged-in Chrome session.
2. Use CDP to find the tab:
   - `/home/lorenzo/git/chrome-cdp-skill/skills/chrome-cdp/scripts/cdp list`
3. Inspect the page structure if needed:
   - `.../scripts/cdp snap <target>`
4. Fill the form through the live page context.
5. Tick the required checkbox:
   - `Record lterenzi@leggedrobotics.com as the email to be included with my response`
6. Submit through the page's `Submit` button.
7. Verify the confirmation page says:
   - `Your response has been recorded.`

Observed required fields:

- `eth_username`
- `last_name`
- `first_name`
- `legi_number`
- `eth_email`
- `termination_date`
- `supervisors`
- `main_supervisor_email`

Observed field expectations:

- `eth_username`: ETH username without prefixes or suffixes
- `legi_number`: format `12-123-123`
- `eth_email`: must be plain `username@ethz.ch`
- `termination_date`: planned thesis submission date / access end date
- `supervisors`: comma-separated names if more than one
- `main_supervisor_email`: supervisor confirmation email

Observed backing form fields in the live DOM:

- `eth_username`: `entry.1648523464`
- `last_name`: `entry.1992375209`
- `first_name`: `entry.1463250709`
- `legi_number`: `entry.452055767`
- `eth_email`: `entry.1045487484`
- `termination_date`: `entry.68168767_{year,month,day}`
- `supervisors`: `entry.2022322873`
- `main_supervisor_email`: `entry.534103135`

Worked test example that succeeded through Chrome CDP:

- `eth_username`: `teststudent`
- `last_name`: `Student`
- `first_name`: `Test`
- `legi_number`: `12-345-678`
- `eth_email`: `teststudent@ethz.ch`
- `termination_date`: `2026-09-24`
- `supervisors`: `Lorenzo Terenzi`
- `main_supervisor_email`: `lterenzi@ethz.ch`

Observed confirmation after submission:

- `Your response has been recorded.`

## Step 6: Notify Fang Nan For GitHub Group Access

First ask or confirm:

- student full name
- exact GitHub username

Send Fang Nan:

- student full name
- GitHub username

Purpose:

- add the student to the student GitHub group

Preferred contact channel:

- Google Chat API, not browser automation

Messaging rule:

- use the Chat API only for explicit read/send actions requested by the user
- do not create background chat watchers, autonomous reply loops, or bot-like monitoring in shared spaces
- when replying as the assistant, write in first person and keep the wording simple

Current reproducible API path:

1. Make sure `gws` auth includes:
   - `https://www.googleapis.com/auth/chat.messages`
   - `https://www.googleapis.com/auth/chat.messages.create`
   - `https://www.googleapis.com/auth/chat.spaces.readonly`
   - `https://www.googleapis.com/auth/chat.memberships.readonly`
   - `https://www.googleapis.com/auth/directory.readonly`
2. Resolve Fang Nan in the Workspace directory:
   - `gws people people searchDirectoryPeople --params '{"query":"Fang Nan","readMask":"names,emailAddresses,metadata","sources":["DIRECTORY_SOURCE_TYPE_DOMAIN_PROFILE"]}'`
3. Use the primary email to resolve the DM:
   - `gws chat spaces findDirectMessage --params '{"name":"users/fannan@leggedrobotics.com"}'`
4. Send the message into the returned DM space with:
   - `gws chat spaces messages create --params '{"parent":"spaces/l5Wmn4AAAAE"}' --json '{"text":"..."}'`

Resolved Fang Nan identity observed on this machine:

- display name: `Fang Nan`
- primary email: `fannan@leggedrobotics.com`
- people resource: `people/105618277422297757832`
- direct-message space: `spaces/l5Wmn4AAAAE`

Worked test message:

- `Hi Fang, could you please add the test student to the student GitHub group? Name: Test Student. GitHub: test-student. This is just a test request while I am validating the onboarding workflow. Thanks.`

Observed result:

- message name: `spaces/l5Wmn4AAAAE/messages/T4WqW64vBeU.T4WqW64vBeU`
- create time: `2026-03-24T22:37:13.221711Z`

## Known Gotchas

- Spreadsheet rows must be inserted at the top; appending is wrong for this sheet.
- The outer project folder and inner student-material folder are different levels.
- The grading sheet must sit in the outer project folder.
- Existing historical folder naming is not fully consistent, so prefer the explicit `MT - Name` / `MA - Name` convention for new folders unless told otherwise.
- The Euler form and the student-PC form are different classes of Google Forms:
  - Euler form: raw `formResponse` automation works
  - student-PC form: signed-in Chrome + CDP is required
- Chat automation here means one explicit assistant action at a time, not an autonomous bot.
- The Fang Nan step should use the Chat API, not browser automation, once the required Chat + Directory scopes are present.
