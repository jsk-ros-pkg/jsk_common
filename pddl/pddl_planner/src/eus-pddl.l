;; utility library for eus to use pddl-planner

;; automatically load roseus and ros message packages
(pushnew (format nil "~A/euslisp/"
                 (read-line (piped-fork "rospack find roseus")))
         *load-path* :test #'equal)
(load "roseus.l")
;; we alse need to load actionlib.l
(load "actionlib.l")

(roseus-add-msgs "roslib")
(roseus-add-msgs "roseus")
(roseus-add-msgs "std_msgs")
(roseus-add-msgs "pddl_msgs")
(roseus-add-actionlibs "pddl_msgs")

;; class
(defclass pddl-module
  :super propertied-object
  :slots ()
  )

;; '((a . <a>) (b . <b>) (aa . <a>))
(defclass pddl-problem
  :super pddl-module
  :slots (problem-name
          initial-condition
          goal-condition
          domain-name
          obj                           ;((name . type) (name . type) ...)
          ))

(defmethod pddl-problem
  (:init (&key (name nil) (domain nil) ((:objects o) nil))
    (setq problem-name name)
    (setq domain-name domain)
    (setq obj o)
    self)
  ;; accessors
  (:name (&optional name)
    (if name (setq problem-name name))
    problem-name)
  (:domain (&optional name)
    (if name (setq domain-name name))
    domain-name)
  (:objects (&optional o)
    (if o (setq obj o))
    obj)
  (:initial-condition (&optional (condition-list nil))
    (if condition-list (setq initial-condition condition-list))
    initial-condition)
  (:goal (&optional (condition-list nil))
    (if condition-list (setq goal-condition condition-list))
    goal-condition)
  (:goal-condition (&optional (condition-list nil)) ;alias to :goal
    (send self :goal condition-list))
  (:problem-sexp ()
    `(define (problem ,problem-name)
       (:domain ,domain-name)
       (:objects
         ,@(let ((types (remove-duplicates (mapcar #'cdr obj))))
             (mapcar #'(lambda (_type)
                         (append
                          (mapcar #'car
                                  (remove-if-not
                                   #'(lambda (x)
                                       (eq (cdr x) _type))
                                   obj))
                          (list '- _type)))
                     types)))
       (:init ,initial-condition)
       (:goal (and ,@goal-condition))))
  (:write-to-file (file)
    (let ((sexp (send self :problem-sexp)))
      (with-open-file (f file :direction :output)
        (pprint sexp f))))
  (:print ()
    (pprint (send self :problem-sexp)))
  (:add (&key ((:initial-condition init) nil)
              ((:goal-condition goal) nil)
              ((:objects o) nil))
    (if init (push init initial-condition))
    (if goal (push goal goal-condition))
    (if o (push o obj))
    self)
  (:ros-message ()
    (instance pddl_msgs::PDDLProblem :init
              :name (string problem-name)
              :domain (string domain-name)
              :objects (mapcar #'(lambda (x)
                                   (instance pddl_msgs::PDDLObject
                                             :init
                                             :type (string (cdr x))
                                             :name (string (car x))))
                               obj)
              ;;we need to avoid `init' slot because of roseus naming rule
              :initial (mapcar #'(lambda (i) (format nil "~A" i))
                               initial-condition)
              :goal (format nil "~A" `(and ,@goal-condition))))
  )

(defclass pddl-domain
  :super pddl-module
  :slots (domain-name
          requirements
          types
          constants
          predicates
          action)
  )
(defmethod pddl-domain
  (:init (&key (name nil) ((:requirements req) nil) (file nil))
    (if name (setq domain-name name))
    (if req (setq requirements req))
    self)
  (:name (&optional n)
    (if n (setq domain-name n))
    domain-name)
  (:requirements (&optional req)
    (if req (setq requirements req))
    requirements)
  (:types (&optional typ)
    (if typ (setq types typ))
    types)
  (:constants (&optional con)
    (if con (setq constants con))
    constants)
  (:predicates (&optional pre) ;;;(at-robby ?r - room) -> (at-robby (?r room))
                               ;;;(at ?b - ball ?r - room) -> (at (?b ball) (?r room))
    (if pre (setq predicates pre))
    predicates)
  (:action (&optional ac)
    (if ac (setq action ac))
    action)
  (:add (&key
         ((:requirements req) nil)
         ((:types typ) nil)
         ((:constants con) nil)
         ((:predicates pre) nil)
         ((:action ac) nil))
    (if req (push-back req requirements))
    (if typ (push-back typ types))
    (if con (push-back con constants))
    (if pre (push-back pre predicates))
    (if ac  (push-back ac action))
    self)
  (:domain-sexp ()
    `(define (domain ,domain-name)
       (:requirements ,@requirements)
       (:types ,@types)
       (:constants ,@constants)
       (:predicates
         ,@predicates)
       ,@(send-all action :action-sexp)
       ))
  (:print ()
    (pprint (send self :domain-sexp)))
  (:write-to-file (fname)
    (with-open-file (f fname :direction :output)
      (pprint (send self :domain-sexp) f)))
  (:ros-message ()
    (instance pddl_msgs::PDDLDomain :init
              :name (string domain-name)
              :requirements (list->simple-string requirements)
              :types (mapcar #'string types)
              ;;:constants (list->simple-string constants) ; not supported
              :predicates
              (mapcar
               #'(lambda (x) (format nil "~A" x))
               predicates)
              :actions (send-all action :ros-message)))
  )

(defclass pddl-action
  :super pddl-module
  :slots (name
          parameters
          precondition
          effect)
  )
(defmethod pddl-action
  (:init (&key ((:name n) nil)
               ((:parameters par) nil)
               ((:precondition pre) nil)
               ((:effect ef) nil))
    (if n (setq name n))
    (if pre (setq precondition pre))
    (if par (setq parameters par))
    (if ef (setq effect ef))
    self)
  (:name (&optional n)
    (if n (setq name n))
    name)
  (:parameters (&optional par)  ;;; (?obj0 - type0 ?obj1 - type1) -> ((?obj0 type0) (?obj1 type1))
                                ;;; (?obj0 ?obj1 - type0 ?obj2 - type1) -> ((?obj ?obj type) (?obj2 type1))
                                ;;; (?obj0 ?obj1) -> ((?obj0) (?obj1))
    (if par (setq parameters par))
    parameters)
  (:precondition (&optional pre)
    (if pre (setq precondition pre))
    precondition)
  (:effect (&optional ef)
    (if ef (setq effect ef))
    effect)
  (:add (&key ((:effect ef) nil)
              ((:parameters par) nil)
              ((:precondition pre) nil)
              )
    (if ef (push-back ef effect))
    (if par (push-back par parameters))
    (if pre (push-back par precondition))
    self)
  (:action-sexp ()
    `(:action ,name
       :parameters ,@(mapcar #'(lambda (x)
                                (if (= (length x) 1)
                                    (list (car x))
                                  (append (butlast x) '(-) (last x))))
                             parameters)
       :precondition (and ,@precondition)
       :effect (and ,@effect)))
  (:ros-message ()
    (instance pddl_msgs::PDDLAction :init
              :name (string name)
              :parameters
              (list->simple-string
               (mapcar #'(lambda (x)
                           (if (= (length x) 1)
                               (list (car x))
                             (append (butlast x) '(-) (last x))))
                       parameters))
              :precondition
              (format nil "~A" `(and ,@precondition))
              :effect
              (format nil "~A" `(and ,@effect))))
  )

;; macro for define domain and problem
(defmacro define-pddl-problem (sym
                               &key
                               name domain objects
                               initial-condition goal-condition)
  (let ((tmp (gensym)))
    `(setq ,sym (let ((,tmp (instance pddl-problem :init)))
                  (send ,tmp :name ',name)
                  (send ,tmp :domain ',domain)
                  (send ,tmp :objects ',objects)
                  (send ,tmp :initial-condition ',initial-condition)
                  (send ,tmp :goal ',goal-condition)
                  ,tmp))))

(defun make-pddl-action (action-spec)
  (destructuring-bind
      (name &key parameters precondition effect)
      action-spec
    (instance pddl-action :init
              :name (string name)
              :parameters parameters
              :precondition precondition
              :effect effect)))

(defmacro define-pddl-domain (sym
                              &key
                              name requirements
                              types predicates
                              actions)
  (let ((tmp (gensym)))
    `(setq ,sym (let ((,tmp (instance pddl-domain :init)))
                  (send ,tmp :name ',name)
                  (send ,tmp :requirements ',requirements)
                  (send ,tmp :types ',types)
                  (send ,tmp :predicates ',predicates)
                  ;; build actions
                  (send ,tmp :action
                        (mapcar #'make-pddl-action ',actions))
                  ,tmp))))

;; function for parse planning result


;; utility
(defun list->simple-string (x)
  "this function is not cool...."
  (let ((str (format nil "~A" x)))
    (subseq str 1 (1- (length str))))) ; remove ( and )
