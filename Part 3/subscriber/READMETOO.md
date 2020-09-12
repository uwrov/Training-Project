# Robot Plugin

Working file for ['wb_plugin.cc`](https://github.com/uwrov/Training-Project/blob/master/src/wb/plugins/wb_plugin.cc).

There's a **lot** going on in `wb_plugin.cc` - so much that it can be pretty intimidating to read through for the first time. We'll try to break down the major sections of the code here.

Again, **do not feel like you have to understand everything**, as long as you come away with a basic understanding of what the code does, that's good enough for now.

## High Level Overview
Here's a quick rundown of what this plugin does.

1. Define an object `WBPlugin`
2. Subscribe to `wheely_boi/wheely_boi/cmd`
3. Use our commands to calculate the force to be applied onto each wheel of wheely_boi
4. Send commands to Gazebo

### 1. Defining `WBPlugin`
This all happens in `wb_plugin.h`. See below for more detail about header files.

### 2. Subscribing

### 3. Calculations

### 4. Sending

## About C++
Let's address the elephant in the room: C++.

C++ is super cool, but we don't have the room in this tutorial to explore why it's cool. Instead, we'll just try to give you *just enough* language specific knowledge required to get a basic understanding of what this program does.

You might be wondering: if we're not going to explain C++, why is this file written in it? Why not Python?

The answer to this question is a sad one: there's no Python interface for Gazebo.

With all that said, let's explore some parts of the file:

### Header Files
You might have noticed that we have two files: `wb_plugin.cc` and `wb_plugin.h`, why have both?

This is a kind of stylistic choice which is tied to necessity.

C/C++ are lazy, and their compilers work through the programs line by line. This means that this Java practice won't work in C/C++:

```c
1   void A() {
2       B();
3   }
4   
5   void B() {
6       System.out.print("hi");
7   }
```
This will throw an error in C/C++, as the compiler won't recognize `B()` as a function when it's first called on line 2 because it hasn't been *declared* yet.

To fix this, we can simply declare `B()` up top:
```
1   void B();
2
3   void A() {
4       B();
5   }
6   
7   void B() {
8       System.out.print("hi");
9   }
```
Now when we call `B()` on line 4, the compiler will know it's a function, and fill in the function *definition* using the code from line 7 onward.

As you can probably guess, this can cause some headaches, so we can put all our function declarations into a **header file**, and just include it at the top of our file.

To make the best of an unfortunate situation, we go a step further and treat the header files as an "interface" or "front facing" side of our code. Another programmer should be able to read our header file and understand what our code is capable of.

Here's a quick list on how well styled code should be divided:

- Header Files (.h, .hh, .hpp, or whatever you want)
  - Class / Function declarations
  - Class / Function comments
  - Macros
  - Imports
- Code Files (.cc)
  - Class / Function definitions
  - Code
  - Inline comments
  - Imports


### Classes
C++ supports objects! If you're familiar with Java objects then you should recognize most of the items.

Here's an example class ClassName's declaration, which extends ParentClass:
```cpp
// This would be within a header file
class ClassName : public ParentClass {
 public:
  ClassName();
  void A();
 private:
  int a_;
  int b_;
};
```

`private` and `public` are basically the same as in Java. `private` data members can't be accessed externally (with some exceptions). `public` data members can be accessed by anyone or anything.

Also note the stylistic choice to append `_` to private data member's names. This is arbitrary (`wb_plugin.cc` has the `_` at the front), but make sure to have something to distinguish your private data members.
 
Now if we want to add definitions, we would do this code:

```cpp
// This would be in a code (.cc) file
void ClassName::A() {
    // code
}
```

And that's the long and short of C++ classes!

### Pointers
Pointers are variables which store the address of other variables.

Pointers are one of the features of C/C++ which allow us to have finer control over our program's memory usage, which is one of the main draws of the language family.

Now examine the following code:
```cpp
// Construct a new ClassName called a
ClassName a = ClassName();

// Make a pointer a_ptr to point to the address of a
ClassName* a_ptr = &a;

// Call a.A()
a.A();

// Call a.A(), but through the pointer to a!
a_ptr->A();
```
This isn't a great example about how pointers *should* be used, but it's a simple example about how pointers *could* be used.

There's a lot more to learn about pointers, but we won't bet getting into them here.

### Namespaces
Let's look at the equivalent of `System.out.println("Hello world")` in C++:
```cpp
std::cout << "Hello world" << std::endl;
```
If we get annoyed of typing `std::`, we can do something like this:
```cpp
namespace std {
cout << "Hello world" << endl;
```