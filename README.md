# 2025-group-12

## Tools used
* Docker 
* Ubuntu 24.04
* git 
* C++
* CMake
* VSCode

## Installation / Step-to-step guide
After cloning the project into a new folder, run the following commands in the terminal to build and run the project:

```bash
cd a6
docker build -t <name_of_image>:latest -f Dockerfile .
docker run --rm <name_of_image>:latest 42
```

## Authors and Acknowledgment
Ling Svahn,
William Johansson, 
Sin Yee Sheenie Chan,
Samuel Partain

## How the team is working
### Adding new features
The team has decided to use feature branches for the development. Each branch will be connected to an issue and from that issue a branch will be created. Each branch will have maximun two developers working on it at the same time, and minimum one developer. Hence, max four branches will be open at the same time, excluding the main/master branch.

Each issue will have a **goal/purpose** and **acceptance criterias** to ensure a mutual understanding between the developers by having a shared understanding of what needs to be done, and when the tasks are completed. The acceptance criterias shall be testable and consist of something the system should do.

When an issue has all acceptance criterias completed, it is ready for a merge request. The developer(s) who has worked on the issue and its related branch are not allowed to be a reviewer of the merge request. The reviewer looks through the changes made and leaves at least one comment on what is positive. Additional comments are not mandatory, if not applicable. If something can be improved, the reviewer leaves constructive critique/comments regarding the code or artifact. The comments shall be about implementation or the artifact and not about a person.

The developer(s) working on the branch will look through the reviewer's comments and do changes if necessary. If no changes are made when the reviewer made suggested improvements the developer(s) have to explain why the suggested improvement has not been done by writing comments in the merge request (i.e., answer to the reviewers comments). 

The reviewer approves after changes have been made or developers’ comments has been sufficient enough why changes has not been applied. The reviewer then merges the branch to main. The source branch shall be deleted and the comments untouched, i.e., do not squash the comments.

If there are no improvement comments which the reviewer can leave ( the artifact is perfect as it is ), then it is encouraged to merge the branch directly. There is no need for the developer(s) to resolve comments if there is nothing to change. 

### Fix unexpected behaviour in existing featurs 
If a problem occurs in the code or artifact, the group shall be notified in the designated communication media. An issue will be created and related branch where the problem will be resolved. When the problem has been resolved, the related branch will be merged to the main/master branch with the procedure mentioned above. 

## Commit style
The commits shall start with # followed by the issue number the commit is connected to. The commit shall be maximum 50 characters long and it shall be written in present tense. A commit shall follow the following structure: If I make this commit I will <the commit message>. No punctuation mark shall be used at the end of the commit message.

An example of a commit message:
#64 Add attributes to class Car

## License 
This project is released under the terms of the MIT License.

