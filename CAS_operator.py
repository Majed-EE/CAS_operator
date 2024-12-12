from flask import Flask, redirect, url_for, render_template, session
from flask import request
from datetime import timedelta
from flask import flash




app=Flask(__name__)
app.secret_key="hello"
app.permanent_session_lifetime=timedelta(minutes=5)


@app.route("/") # decorator to give the route
def home():
    return render_template("index.html")


@app.route("/login",methods=["POST","GET"])
def login():
    if request.method=="POST":
        session.permanent=True
        user=request.form["nm"]
        session["user"]=user
        flash("You have been logged in!", "info")
        print(f"type of session is {type(session)}")
        print(f"session is \n{session}") #<SecureCookieSession {'key': 'value'}>
        return redirect(url_for("user", usr=user))
    else:
        if "user" in session:
            flash("Already logged in!", "info")
            return redirect(url_for("user")) # redirect for is the name of the data
        return render_template("login.html")

@app.route("/user")
def user():
    if "user" in session:
        user=session["user"]
        flash("hello user", "info")
        return render_template("user.html",user=user)
    else:
        flash("You are not logged in!", "info")
        return redirect(url_for("login"))
    

@app.route("/logout")
def logout():
    session.pop("user", None)
    flash("You have been logged out!", "info")
    print("flash")
    return redirect(url_for("login"))


if __name__=="__main__":
    app.run(debug=True)















# @app.route("/<name>")
# def user(name):
#     return render_template("index.html",content=name)


# @app.route("/admin")
# def admin():
#     return redirect(url_for("user",name="Admin!"))
