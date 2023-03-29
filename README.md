# RT2_Sphinx-Documentation

 <img src="https://user-images.githubusercontent.com/62358773/158238820-f418cc09-4227-4afc-9c31-1705dfb64f5a.png" width="5%" height="5%"> Professor [Carmine Recchiuto](https://github.com/CarmineD8), <img src="https://user-images.githubusercontent.com/62358773/158238810-c5dcb486-ba24-4b35-87de-39a54e88f36b.png" width="5%" height="5%"> Student: [Subhransu Sourav Priyadarshan](https://github.com/subhransu10)

## AIM 
 In this assignment,I developed documentation for [RT 1 - Assignment 3](https://github.com/subhransu10/RT1_Assignment3). In order to achieve this task, I used __Sphinx__ tool .
 
## INTRODUCTION 
In this documentation, [Sphinx](https://www.sphinx-doc.org/en/master/) tool is used. Sphinx was originally created for Python, but it has now facilities for the documentation of software projects in a range of languages. 

## INSTALLATION
In order to install, we write the following command:
```bash
sudo apt-get install python3-sphinx
```
After installing Sphinx, we need to type the following command:
```bash
sphinx-quickstart
```
After running the command, accept the defaults. Then, write the Project Name, Author Name, Project Release.

## PREPARING THE DOCUMENTATION
After installing __Sphinx__ and following the above mentioned steps, we can find the __conf.py__ file in your directory path but we still need to add a few things.
```python
extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]
```
```python
extensions = [
'sphinx.ext.autodoc',
'sphinx.ext.doctest',
'sphinx.ext.intersphinx',
'sphinx.ext.todo',
'sphinx.ext.coverage',
'sphinx.ext.mathjax',
'sphinx.ext.ifconfig',
'sphinx.ext.viewcode',
'sphinx.ext.githubpages',
"sphinx.ext.napoleon",
'sphinx.ext.inheritance_diagram',
'breathe'
]
```
```python
highlight_language = 'c++'
source_suffix = '.rst'
master_doc = 'index'
html_theme = 'sphinx_rtd_theme'
```
```python
# -- Extension configuration -------------------------------------------------
# -- Options for intersphinx extension ---------------------------------------
# Example configuration for intersphinx: refer to the Python standard library.
intersphinx_mapping = {'https://docs.python.org/': None}
# -- Options for todo extension ----------------------------------------------
# If true, `todo` and `todoList` produce output, else they produce nothing.
todo_include_todos = True
# -- Options for breathe
breathe_projects = {
"turtlesim_controller": "_build/xml/"
}
breathe_default_project = "turtlesim_controller"
breathe_default_members = ('members', 'undoc-members')
```

After making all the changes in __conf.py__ file, run the command given below:
```bash
doxywizard
```
The command to install Doxygen is :
```bash
sudo apt-get install –y doxygen
sudo apt-get install doxygen-gui
```
Finally,I modified the __index.rst__ script, which will be used by sphinx (together with conf.py and (indirectly) with Doxygen.in, to build our documentation). Then I ran the command
```bash
make html
```
The command will create the __index.html__ file.


## RESULT

Finally, I need to update my documentation online on GitHub, so that it could be visualized by people using repository.

Use GitHub to publish your doucmentation online. For this, follow the steps given below:

Because it's a Sphinx documentation,so i added an empty file, in the __docs folder__, named ".nojekyll" (this is needed for using the sphinx layout).
Lastly,I went to Settings of GitHub repository (for which I created documentation and need to create website link for the same) -> Pages.
On Pages section,I made changes under GitHub Pages -> Branch.Then, I changed branch from none to main, and /root folder to /docs. Finally,I saved the settings and  an url got activated to visualize the documentation.

To view the documentation visit the link given below:

(https://subhransu10.github.io/RT2_Sphinx-Documentation/)

This the url for the Sphinx Documentaion.
