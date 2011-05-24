;; roslaunch-popup.el
;;
;; this package depends on popup.el.
;; You can get popup.el
;; from http://cx4a.org/pub/auto-complete/auto-complete-1.2.tar.bz2
;;
;; features are:
;;  1) poping-up the contents of the launch file specified by
;;     include tag around the pointer using  popup.el
;;  2) open the launch file specified by include tag near the pointer.
;;
;; You need to write some codes to use this package:
;; --------------------------------------------------
;;  ;; Needless to say, you need to add roslaunch-popup.el and popup.el to
;;  ;; your load-path.
;;  (require 'roslaunch-popup)
;;  ;; i recomend to bind roslaunch-popup-popup, roslaunch-popup-open
;;  ;; and roslaunch-popup-open-subwindow to some key bind like below:
;;  (global-set-key "\M-[" 'roslaunch-popup-popup)
;;  (global-set-key "\M-]" 'roslaunch-popup-open-subwindow)
;; --------------------------------------------------
;;
;; roslaunch-popup.el is written by R.Ueda.
;;  contact: ueda@jsk.t.u-tokyo.ac.jp

(require 'popup)
(require 'cl)

(defmacro roslaunch-popup-with-temp-buffer (buf &rest args)
  "This macro takes a buffer `buf', copy the texts of `buf'
to a temporary buffer, and evaluate `args' with binding the temporary buffer
to current-buffer. The return value is the last value of `args'."
  (let ((b (gensym)))
    `(let ((,b ,buf))
       (with-temp-buffer
         (insert-buffer-substring ,b)
         ,@args))
    ))

(defmacro* roslaunch-popup-kill-buffer-if-not-visited ((buf bufname) &rest args)
  "This macro takes buffer name and an object of buffer.
First, this macro checks if there is the buffer whose name is equal to bufname.
If there is, this macro do nothing, just eval `args' with binding `buf'
to `current-buffer'.
If there isn't, this macro call `kill-buffer' without any warnings after
evaluate `args'.
NB: This macro test by calling `get-buffer'"
  (let ((buffer-exist-p (gensym))
        (target-buffer (gensym)))
    `(let ((,buffer-exist-p (get-buffer ,bufname)))
       (let ((,target-buffer ,buf))
         (unwind-protect
             (with-current-buffer ,target-buffer
               ,@args)
           (unless ,buffer-exist-p
             (with-current-buffer ,target-buffer
               (set-buffer-modified-p nil))
             (kill-buffer ,target-buffer))
           )))
    ))

(defmacro* roslaunch-popup-kill-file-buffer-if-not-visited ((file-name) &rest args)
  "This macro is the specialized macro of `org-blog-kill-buffer-if-not-visited'.
This macro takes only `file-name'. This macro is required because, file-buffer
returns the file name without directory paths by `get-buffer' as the name, but
we want to check by absolute path."
  (let ((buffer (gensym))
        (buffer-exist-p (gensym)))
    `(let ((,buffer-exist-p (get-file-buffer (expand-file-name ,file-name))))
       ;; if you already visit the file, ,buffer-exist-p is an instance
       ;; of buffer.
       (let ((,buffer (or ,buffer-exist-p (find-file-noselect ,file-name))))
         (unwind-protect
             (with-current-buffer ,buffer
               ,@args)
           (unless ,buffer-exist-p
             (with-current-buffer ,buffer
               (set-buffer-modified-p nil))
             (kill-buffer ,buffer)))))
    ))


(defvar roslaunch-popup-find-regexp "^\$(find \\(.*\\))")

(defun roslaunch-popup-path-using-find-p (str)
  "check str starts with the string '$(find'"
  (string-match roslaunch-popup-find-regexp str))

(defun roslaunch-popup-replace-find-path (str)
  (let ((n (string-match roslaunch-popup-find-regexp str)))
    (let ((find-string (match-string 1 str)))
      (replace-regexp-in-string
       roslaunch-popup-find-regexp
       (concat
        (replace-regexp-in-string
         "\n$" ""
         (shell-command-to-string 
          (format "rospack find %s" find-string))) "")
       str t))))

(defun roslaunch-popup-resolve-path (str)
  "This function take a string like $(find hoge)/fuga/piyo.launch, and returns
a string like path-to-hoge/fuga/piyo.launch using `rospack' command."
  (interactive "s")
  (if (roslaunch-popup-path-using-find-p str)
      (roslaunch-popup-replace-find-path str)
    str))
;;(shell-command-to-string "rospack find")

(defun roslaunch-popup-tag-string-near-pointer ()
  (interactive)
  (let ((current-pos (point)))
    (unwind-protect
        (progn
          (re-search-backward
           "<[^-]" nil t)
          (goto-char (match-beginning 0))
          (let ((from (point)))
            (re-search-forward
             "[^-]>" nil t)
            (goto-char (match-end 0))
            (let ((to (point)))
              (buffer-substring from to))))
      (goto-char current-pos))))

(defun roslaunch-popup-have-include-tag-p (str)
  "check the str start with <include"
  (interactive "s")
  ;; need to add whitespace groups to the end
  (string-match "^<include" str))

(defun roslaunch-popup-have-file-attribute-p (str)
  "check the str has the string file=\"\""
  (interactive "s")
  (string-match "file=\".*\"" str))

(defun roslaunch-popup-funcall-program (func)
  (let ((str (roslaunch-popup-tag-string-near-pointer)))
    (when (and (roslaunch-popup-have-include-tag-p str)
               (roslaunch-popup-have-file-attribute-p str))
      ;; str = "<include ...>"
      (let ((file=string (let ((n (string-match "file=\"\\(.*\\)\"" str)))
                           (match-string 1 str))))
        (let ((launch-fname (roslaunch-popup-resolve-path file=string)))
          (funcall func launch-fname))))))

(defun roslaunch-popup-popup ()
  "This function needs to be called interactively. When the marker is
located inside the include tag, popup the contents of the included launch file
with popup-tip in popup.el."
  (interactive)
  (roslaunch-popup-funcall-program
  #'(lambda (launch-fname)
      (message "popping up %s" launch-fname)
      (let ((contents 
             (roslaunch-popup-kill-file-buffer-if-not-visited
              (launch-fname)
              (buffer-substring (point-min) (point-max)))))
        (popup-tip contents)))))

(defun roslaunch-popup-open ()
  "This function needs to be called interactively. When the marker is
located inside the include tag, open the included launch file"
  (interactive)
  (roslaunch-popup-funcall-program #'find-file))

(defun roslaunch-popup-open-subwindow ()
  "This function needs to be called interactively. When the marker is
located inside the include tag, open the included launch file"
  (interactive)
  (roslaunch-popup-funcall-program #'find-file-other-window))


(provide 'roslaunch-popup)
