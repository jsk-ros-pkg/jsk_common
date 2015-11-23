;; -*- mode: Emacs-Lisp -*-
;; written by k-okada 2006.06.14
;;
;; changed by ueda 2009.04.21

;; (debian-startup 'emacs21)

;;; Global Setting Key
;;;
(global-set-key "\C-h" 'backward-delete-char)
(global-set-key "\M-g" 'goto-line)
(global-set-key "\C-xL" 'goto-line)
(global-set-key "\C-xR" 'revert-buffer)
(global-set-key "\er" 'query-replace)

(global-unset-key "\C-o" )

(setq visible-bell t)

;;; When in Text mode, want to be in Auto-Fill mode.
;;;
(when nil
  (defun my-auto-fill-mode nil (auto-fill-mode 1))
  (setq text-mode-hook 'my-auto-fill-mode)
  (setq mail-mode-hook 'my-auto-fill-mode))

;;; time
;;;
(load "time" t t)
(display-time)

;;; for mew
;;;
;;;     explanation exists in /opt/src/Solaris/mew-1.03/00install.jis
;;;
(when nil ;; obsolate mew setting
(autoload 'mew "mew" nil t)
(autoload 'mew-read "mew" nil t)
(autoload 'mew-send "mew" nil t)

(setq mew-name "User Name")
(setq mew-user "user")
(setq mew-dcc "user@jsk.t.u-tokyo.ac.jp")

(setq mew-mail-domain "jsk.t.u-tokyo.ac.jp")
(setq mew-pop-server "mail.jsk.t.u-tokyo.ac.jp")
(setq mew-pop-auth 'apop)
(setq mew-pop-delete 3)
(setq mew-smtp-server "mail.jsk.t.u-tokyo.ac.jp")
(setq mew-fcc nil)
(setq mew-use-cached-passwd t)

;;
;; Optional setup (Read Mail menu for Emacs 21):
(if (boundp 'read-mail-command)
    (setq read-mail-command 'mew))

;; Optional setup (e.g. C-xm for sending a message):
(autoload 'mew-user-agent-compose "mew" nil t)
(if (boundp 'mail-user-agent)
    (setq mail-user-agent 'mew-user-agent))
(if (fboundp 'define-mail-user-agent)
    (define-mail-user-agent
      'mew-user-agent
      'mew-user-agent-compose
      'mew-draft-send-message
      'mew-draft-kill
      'mew-send-hook))

(define-key ctl-x-map "r" 'mew)
(define-key ctl-x-map "m" 'mew-send)
) ;; /obsolate mew setting

;; (lookup)
;;
(setq lookup-search-agents '((ndtp "nfs")))
(define-key global-map "\C-co" 'lookup-pattern)
(define-key global-map "\C-cr" 'lookup-region)
(autoload 'lookup "lookup" "Online dictionary." t nil )

;; Japanese
;; uncommented by ueda. beacuse in shell buffer, they invokes mozibake
(set-language-environment 'Japanese)
(prefer-coding-system 'utf-8)
(setq enable-double-n-syntax t)

(setq use-kuten-for-period nil)
(setq use-touten-for-comma nil)

;; sudo apt-get install yc-el migemo
(when (require 'yc nil t)
  (load-library "yc"))
;; (when (require 'migemo nil t)
;;   (load "migemo"))

;;; Timestamp
;;;
(defun timestamp-insert ()
  (interactive)
  (insert (current-time-string))
  (backward-char))
(global-set-key "\C-c\C-d" 'timestamp-insert)

(global-font-lock-mode t)

;; M-n and M-p
(global-unset-key "\M-p")
(global-unset-key "\M-n")

(defun scroll-up-in-place (n)
       (interactive "p")
       (previous-line n)
       (scroll-down n))
(defun scroll-down-in-place (n)
       (interactive "p")
       (next-line n)
       (scroll-up n))

(global-set-key "\M-n" 'scroll-down-in-place)
(global-set-key "\M-p" 'scroll-up-in-place)

;; dabbrev
(global-set-key "\C-o" 'dabbrev-expand)

;; add by kojima
(require 'paren)
(show-paren-mode 1)
;; ;; C-qで移動
(defun match-paren (arg)
  "Go to the matching parenthesis if on parenthesis."
  (interactive "p")
  (cond ((looking-at "\\s\(") (forward-list 1) (backward-char 1))
        ((looking-at "\\s\)") (forward-char 1) (backward-list 1))
        )
  )
(global-set-key "\C-Q" 'match-paren)

(font-lock-add-keywords 'lisp-mode
                        (list
                          (list "\\(\\*\\w\+\\*\\)\\>"
                                '(1 font-lock-constant-face nil t))
                          (list "\\(\\+\\w\+\\+\\)\\>"
                                '(1 font-lock-constant-face nil t))))

(when t
;; does not allow use hard tab.
(setq-default indent-tabs-mode nil)
)

;; ignore start message
(setq inhibit-startup-message t)

;; shell mode
(set-terminal-coding-system 'utf-8)
(set-buffer-file-coding-system 'utf-8)
(setq explicit-shell-file-name shell-file-name)
(setq shell-command-option "-c")
(setq system-uses-terminfo nil)
(setq shell-file-name-chars "~/A-Za-z0-9_^$!#%&{}@`'.,:()-")
(add-hook 'shell-mode-hook 'ansi-color-for-comint-mode-on)


(defun lisp-other-window ()
  "Run lisp on other window"
  (interactive)
  (if (not (string= (buffer-name) "*inferior-lisp*"))
      (switch-to-buffer-other-window
       (get-buffer-create "*inferior-lisp*")))
  (run-lisp inferior-euslisp-program))

(set-variable 'inferior-euslisp-program "roseus")
(global-set-key "\C-cE" 'lisp-other-window)

;; add color space,tab,zenkaku-space
(unless (and (boundp '*do-not-show-space*) *do-not-show-space*)
  (defface my-face-b-1 '((t (:background "gray"))) nil)
  (defface my-face-b-2 '((t (:background "red"))) nil)
  (defface my-face-u-1 '((t (:background "red"))) nil)
  (defvar my-face-b-1 'my-face-b-1)
  (defvar my-face-b-2 'my-face-b-2)
  (defvar my-face-u-1 'my-face-u-1)

  (defadvice font-lock-mode (before my-font-lock-mode ())
    (font-lock-add-keywords
     major-mode
     '(
       ("\t" 0 my-face-b-1 append)
       ("　" 0 my-face-b-2 append)
       ("[ \t]+$" 0 my-face-u-1 append)
       )))
  (ad-enable-advice 'font-lock-mode 'before 'my-font-lock-mode)
  (ad-activate 'font-lock-mode)
  )
;; to change indent for euslisp's method definition ;; begin
(define-derived-mode euslisp-mode lisp-mode
  "EusLisp"
  "Major Mode for EusLisp"
  )
(defun lisp-indent-function (indent-point state)
  "This function is the normal value of the variable `lisp-indent-function'.
It is used when indenting a line within a function call, to see if the
called function says anything special about how to indent the line.

INDENT-POINT is the position where the user typed TAB, or equivalent.
Point is located at the point to indent under (for default indentation);
STATE is the `parse-partial-sexp' state for that position.

If the current line is in a call to a Lisp function
which has a non-nil property `lisp-indent-function',
that specifies how to do the indentation.  The property value can be
* `defun', meaning indent `defun'-style;
* an integer N, meaning indent the first N arguments specially
  like ordinary function arguments and then indent any further
  arguments like a body;
* a function to call just as this function was called.
  If that function returns nil, that means it doesn't specify
  the indentation.

This function also returns nil meaning don't specify the indentation."
  (let ((normal-indent (current-column)))
    (goto-char (1+ (elt state 1)))
    (parse-partial-sexp (point) calculate-lisp-indent-last-sexp 0 t)
    (if (and (elt state 2)
             (not (looking-at "\\sw\\|\\s_")))
        ;; car of form doesn't seem to be a symbol
        (progn
          (if (not (> (save-excursion (forward-line 1) (point))
                      calculate-lisp-indent-last-sexp))
                (progn (goto-char calculate-lisp-indent-last-sexp)
                       (beginning-of-line)
                       (parse-partial-sexp (point)
                                           calculate-lisp-indent-last-sexp 0 t)))
            ;; Indent under the list or under the first sexp on the same
            ;; line as calculate-lisp-indent-last-sexp.  Note that first
            ;; thing on that line has to be complete sexp since we are
          ;; inside the innermost containing sexp.
          (backward-prefix-chars)
          (current-column))
      (let ((function (buffer-substring (point)
                                        (progn (forward-sexp 1) (point))))
            method)
        (setq method (or (get (intern-soft function) 'lisp-indent-function)
                         (get (intern-soft function) 'lisp-indent-hook)))
        (cond ((or (eq method 'defun)
                   (and
                    (eq major-mode 'euslisp-mode)
                    (string-match ":.*" function))
                   (and (null method)
                        (> (length function) 3)
                        (string-match "\\`def" function)))
               (lisp-indent-defform state indent-point))
              ((integerp method)
               (lisp-indent-specform method state
                                     indent-point normal-indent))
              (method
                (funcall method indent-point state)))))))
;; to change indent for euslisp's method definition ;; end
;;Xwindow setting

(when nil
(set-foreground-color "white")
(set-background-color "black")
(set-scroll-bar-mode 'right)
(set-cursor-color "white")
)
;;
(line-number-mode t)
(column-number-mode t)

(when nil
;; stop auto scroll according to cursol
(setq comint-scroll-show-maximum-output nil)
)

(setq ring-bell-function 'ignore)
(setq auto-mode-alist (cons (cons "\\.launch$" 'xml-mode) auto-mode-alist))

;; sudo apt-get install rosemacs-el
(when (require 'rosemacs nil t)
  (invoke-rosemacs)
  (global-set-key "\C-x\C-r" ros-keymap))

;; vrml mode
(add-to-list 'load-path (format "%s/.emacs.d" (getenv "HOME")))
(when (file-exists-p (format "%s/.emacs.d/vrml-mode.el" (getenv "HOME")))
  (load "vrml-mode.el")
  (autoload 'vrml-mode "vrml" "VRML mode." t)
  (setq auto-mode-alist (append '(("\\.wrl\\'" . vrml-mode))
                                auto-mode-alist)))

;; matlab mode
(when (file-exists-p (format "%s/.emacs.d/matlab/matlab.el.1.10.1" (getenv "HOME")))
  (load "matlab/matlab.el.1.10.1" (getenv "HOME"))
  (setq auto-mode-alist (append '(("\\.m\\'" . matlab-mode))
                                auto-mode-alist)))

;; for Arduino
(setq auto-mode-alist (append '(("\\.pde\\'" . c++-mode))
                              auto-mode-alist))

;; yaml mode
(when (require 'yaml-mode nil t)
  (add-to-list 'auto-mode-alist '("¥¥.yml$" . yaml-mode)))
