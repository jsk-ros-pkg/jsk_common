# Pull Request
Please write good title when you create a pull request.
**Do not use a title like `update`.**

If you are not confident about writing good title, write like this:

```
[${package}/${program_name}] short description
```

We also recommend you to attach a picture in a pull request.

## CI Error
You may find ci error of your pull-request.

**Please check build log carefully.**

# Commit log
jsk-ros-pkg repositories has a lot of packages in one repository.
You must write easy-to-understood commit log.
Your commit log should be easy-to-understood in `git log --oneline`.


Your commit log must be clear about:

1. Objective. Why needed.
2. Which program?

If you are not confident about writing good commit log, write like this:

```
[${package}/${program_name}] short description

concrete description

Modified:
  - modified_file
...
```

We have [`$ git jsk-commit`](https://github.com/jsk-ros-pkg/jsk_common/blob/master/jsk_tools/bin/git-jsk-commit) command to help you to follow above style of commit message.


# Coding Style
**Please follow other codes in the same package.**


For reference, most of C++ code in jsk-ros-pkg follows the style described below.
It's not a rule, just a indication.
* Use soft tab, do not use hard tab.
* Keep 80-columns as much as possible.
* Do not write `using namespace ...` in headers.
* Do not use pointer, use `boost::shared_ptr`
* Do not use c++1x for old environment. Supporting old environment is better than
writing cool code.
* Use `@` for doxygen markdup instead of `\`

  ```c++
  /**
   * @brief
   * This is an awesome class
   **/
   class Foo
   {
     ...
   };
  ```
* if

   ```c++
   if (test) {
     awesome_code
   }
   else if (test2) {
     awesome_code
   }
   else {
     awesome_code
   }
   ```
* class

   ```c++
   class Foo: public class Bar
   {
     public:
       ...
     protected:
       bool fooBarBar();
       int foo_;
       int foo_bar_;
     private:
       ...
   };
   ```
   Class name should be camel-case and starting with upper case.
   Member variables is snake-case and should have `_` suffix.
   Method should be camel-case and starting with lower case.
* include guard

   ```c++
   #ifndef PACKAGE_NAME_HEADER_FILE_NAME_H_
   #define PACKAGE_NAME_HEADER_FILE_NAME_H_
   ...
   #endif
   ```
* Function and method

  ```c++
  int foo() {
    int bar_bar = 0;
    return 0;
  }
  ```

  Local variables in a function should be snake-case.
